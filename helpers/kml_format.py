
from shapely.geometry import Polygon, MultiPolygon, Point
from lxml import etree
import simplekml
import zipfile # Needed for KMZ

TOLERANCE = 1e-9  # Tolerance for floating point comparisons

#
# KML/KMZ file format handling
#

def read_kml_polygons(kml_file):
    """
    Read ALL polygons from a KML or KMZ file.
    Returns a list of tuples, where each tuple is (name, polygon_geometry).
    """
    doc = None
    # Check file extension
    if kml_file.lower().endswith('.kmz'):
        print(f"Reading KMZ file: {kml_file}")
        try:
            with zipfile.ZipFile(kml_file, 'r') as archive:
                # Find the main KML file within the KMZ
                kml_filename = None
                # Prioritize doc.kml if it exists
                if 'doc.kml' in archive.namelist():
                     kml_filename = 'doc.kml'
                else:
                    # Look for the first *.kml file
                    for name in archive.namelist():
                        if name.lower().endswith('.kml'):
                            kml_filename = name
                            break
                
                if kml_filename:
                    print(f"  Found KML content in: {kml_filename}")
                    doc = archive.read(kml_filename)
                else:
                    raise ValueError("No .kml file found within the KMZ archive.")
        except zipfile.BadZipFile:
             raise ValueError(f"Error: Input file '{kml_file}' is not a valid KMZ file.")
        except Exception as e:
             raise ValueError(f"Error reading KMZ file '{kml_file}': {e}")

    elif kml_file.lower().endswith('.kml'):
        print(f"Reading KML file: {kml_file}")
        try:
            with open(kml_file, 'rb') as f:
                doc = f.read()
        except Exception as e:
             raise ValueError(f"Error reading KML file '{kml_file}': {e}")
    else:
         raise ValueError(f"Error: Input file '{kml_file}' must be a .kml or .kmz file.")

    if doc is None:
         raise ValueError(f"Error: Could not read KML content from '{kml_file}'.")

    # Parse the KML content (from either KML or extracted from KMZ)
    try:
        root = etree.fromstring(doc)
    except etree.XMLSyntaxError as e:
        raise ValueError(f"Error parsing KML content from '{kml_file}': {e}")
        
    ns = {'kml': 'http://www.opengis.net/kml/2.2'}
    
    placemark_elements = root.findall('.//kml:Placemark', namespaces=ns)
    
    all_polygons_data = [] # Store tuples of (name, polygon_geometry)
    unnamed_count = 0

    for placemark in placemark_elements:
        name_elem = placemark.find('./kml:name', namespaces=ns)
        poly_elem = placemark.find('./kml:Polygon', namespaces=ns)
        
        if poly_elem is not None:
            coords_elem = poly_elem.find('.//kml:coordinates', namespaces=ns)
            if coords_elem is not None:
                coords_text = coords_elem.text.strip()
                coords = []
                for coord in coords_text.split():
                    parts = coord.split(',')
                    if len(parts) >= 2:
                        lon, lat = float(parts[0]), float(parts[1])
                        coords.append((lon, lat))
                
                if len(coords) >= 3:
                    polygon = Polygon(coords)
                    if polygon.is_valid:
                        if name_elem is not None and name_elem.text:
                            name = name_elem.text
                        else:
                            unnamed_count += 1
                            name = f"UnnamedPolygon_{unnamed_count}"
                            print(f"Warning: Found unnamed polygon, assigning name: {name}")
                        all_polygons_data.append((name, polygon))

    if not all_polygons_data:
        print(f"Warning: No valid polygons found in the KML content of '{kml_file}'.")
        # Allow returning empty list instead of raising error?
        # raise ValueError("No valid polygons found in KML file.") 
    
    print(f"Read {len(all_polygons_data)} total polygons from KML content.")
    return all_polygons_data


def save_path_to_kml(final_stitched_path_deg, output_file, output_prefix, reverse_path=False, target_polygons_with_names=None, obstacles_with_names=None):
    """
    Saves the final stitched path, target polygons, and obstacle polygons to a KML or KMZ file.
    Polygons are named based on their original name with suffixes.
    Optionally reverses the path points.
    NO FOLDERS are used in the output KML.
    
    Args:
        final_stitched_path_deg: The single final stitched LineString path (degrees) or None.
        output_file: The name of the KML or KMZ file to save.
        output_prefix: The prefix used for naming the track inside the KML (derived from output filename).
        reverse_path: If True, reverse the order of points in the saved path.
        target_polygons_with_names: List of tuples (name, Polygon/MultiPolygon) for targets (degrees).
        obstacles_with_names: List of tuples (name, Polygon) for obstacles (degrees).
    """
    kml = simplekml.Kml()
    kml.document.name = f"{output_prefix}" 
    
    # --- Define Styles (Keep these) --- 
    target_style = simplekml.Style()
    target_style.polystyle.color = simplekml.Color.changealphaint(77, simplekml.Color.green) # ~30%
    target_style.polystyle.outline = 0
    target_style.linestyle.width = 0

    obstacle_style = simplekml.Style()
    obstacle_style.polystyle.color = simplekml.Color.changealphaint(120, simplekml.Color.red) # ~47%
    obstacle_style.polystyle.outline = 0
    obstacle_style.linestyle.width = 0

    # --- Save Final Stitched Path (directly under document) --- 
    if final_stitched_path_deg and final_stitched_path_deg.is_valid and final_stitched_path_deg.coords:
        track_name = output_prefix
        ls = kml.newlinestring(name=track_name)
        
        coords_list = list(final_stitched_path_deg.coords)
        
        if reverse_path:
            print("Reversing final path points before saving.")
            coords_list.reverse()
            
        ls.coords = [(lon, lat) for lon, lat in coords_list] 
        # --- Update Path Style --- 
        # Match example.kml path style: Yellow, 50% opacity, width 2
        # KML color format is aabbggrr (alpha, blue, green, red)
        # 50% alpha is 80 hex (128 dec)
        # Yellow is 00ffff
        # Combined: 8000ffff
        ls.style.linestyle.color = simplekml.Color.changealphaint(128, simplekml.Color.yellow) # Yellow, 50% alpha
        ls.style.linestyle.width = 2 # Width 2
        # --- End Update Path Style ---
        ls.altitudemode = simplekml.AltitudeMode.clamptoground

        # --- Add Start and Finish Placemarks (directly under document) --- 
        if len(coords_list) >= 1: 
            start_coord = coords_list[0]
            # Ensure point is added directly to kml object
            start_pnt = kml.newpoint(name="start_point", coords=[(start_coord[0], start_coord[1])])
            start_pnt.altitudemode = simplekml.AltitudeMode.clamptoground
            start_pnt.style.iconstyle.icon.href = 'http://maps.google.com/mapfiles/kml/paddle/S.png'
            
            if len(coords_list) >= 2:
                finish_coord = coords_list[-1]
                if Point(start_coord).distance(Point(finish_coord)) > TOLERANCE:
                    # Ensure point is added directly to kml object
                    finish_pnt = kml.newpoint(name="finish_point", coords=[(finish_coord[0], finish_coord[1])])
                    finish_pnt.altitudemode = simplekml.AltitudeMode.clamptoground
                    finish_pnt.style.iconstyle.icon.href = 'http://maps.google.com/mapfiles/kml/paddle/F.png' 
                else:
                    start_pnt.name = "Start/Finish" 
            else: 
                 start_pnt.name = "Start/Finish"

    else:
         print("Final stitched path is not available or invalid. Path and points not added to KML.")

    # --- Add Target Polygons (directly under document) --- 
    if target_polygons_with_names: 
        for name, poly_deg in target_polygons_with_names:
             if isinstance(poly_deg, Polygon):
                 polys_to_add = [(name, poly_deg)]
             elif isinstance(poly_deg, MultiPolygon):
                 polys_to_add = [(f"{name}_{j+1}", p) for j, p in enumerate(poly_deg.geoms)]
             else:
                 polys_to_add = []
                 print(f"Warning: Target geometry '{name}' is neither Polygon nor MultiPolygon, skipping.")

             for poly_name, p in polys_to_add:
                 if p.is_valid and p.exterior:
                     # Ensure polygon is added directly to kml object
                     poly_kml = kml.newpolygon(name=f"{poly_name}_target") 
                     poly_kml.outerboundaryis = list(p.exterior.coords)
                     interiors = []
                     for interior in p.interiors:
                          interiors.append(list(interior.coords))
                     if interiors:
                          poly_kml.innerboundaryis = interiors
                     poly_kml.style = target_style 
                     poly_kml.altitudemode = simplekml.AltitudeMode.clamptoground

    # --- Add Obstacle Polygons (directly under document) --- 
    if obstacles_with_names: 
        for name, obs_deg in obstacles_with_names:
            if isinstance(obs_deg, Polygon) and obs_deg.is_valid and obs_deg.exterior:
                 # Ensure polygon is added directly to kml object
                 poly_kml = kml.newpolygon(name=f"{name}_obstacle")
                 poly_kml.outerboundaryis = list(obs_deg.exterior.coords)
                 interiors = []
                 for interior in obs_deg.interiors:
                      interiors.append(list(interior.coords))
                 if interiors:
                      poly_kml.innerboundaryis = interiors
                 poly_kml.style = obstacle_style 
                 poly_kml.altitudemode = simplekml.AltitudeMode.clamptoground
            elif isinstance(obs_deg, MultiPolygon):
                 for j, p in enumerate(obs_deg.geoms):
                      if isinstance(p, Polygon) and p.is_valid and p.exterior:
                           # Ensure polygon is added directly to kml object
                           poly_kml = kml.newpolygon(name=f"{name}_{j+1}_obstacle")
                           poly_kml.outerboundaryis = list(p.exterior.coords)
                           interiors = []
                           for interior in p.interiors:
                                interiors.append(list(interior.coords))
                           if interiors:
                                poly_kml.innerboundaryis = interiors
                           poly_kml.style = obstacle_style
                           poly_kml.altitudemode = simplekml.AltitudeMode.clamptoground
            else:
                 print(f"Warning: Skipping invalid or non-polygon obstacle '{name}'.")

    # Save the KML or KMZ file based on extension
    try:
        if output_file.lower().endswith('.kmz'):
            print(f"Saving as KMZ: {output_file}")
            kml.savekmz(output_file)
        elif output_file.lower().endswith('.kml'):
            print(f"Saving as KML: {output_file}")
            kml.save(output_file)
        else:
             # Default to KML if extension is unknown or missing, maybe add warning?
             print(f"Warning: Unknown output file extension for '{output_file}'. Saving as KML.")
             kml.save(output_file)
    except Exception as e:
        # The exception 'e' might be the NameError if save fails internally
        # Or it could be another error during saving.
        print(f"Error saving KML/KMZ file: {e}")


