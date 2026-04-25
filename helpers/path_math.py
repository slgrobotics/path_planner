
import numpy as np
from shapely.geometry import Polygon, LineString, MultiPolygon, Point, MultiLineString, MultiPoint, GeometryCollection
from shapely.ops import unary_union, transform
from shapely.affinity import rotate
import pyproj

TOLERANCE = 1e-9  # Tolerance for floating point comparisons


def generate_boustrophedon_path(main_polygon_deg, separation_meters=0.3, angle_deg=90.0, safety_distance=0.0):
    """
    Generate a boustrophedon path at a specified angle and separation.
    Projects to UTM, rotates, calculates path, rotates back.
    Returns the path in PROJECTED coordinates (UTM), plus the centroid and transformers.
    """
    # --- Coordinate System Setup & Projection --- 
    crs_deg = "EPSG:4326"
    
    # Reverted: CRS determination should be handled by the caller if multiple polygons are involved
    # The function itself determines CRS based *only* on the input geometry
    if isinstance(main_polygon_deg, MultiPolygon):
        centroid_deg = main_polygon_deg.centroid
    elif isinstance(main_polygon_deg, Polygon):
        centroid_deg = main_polygon_deg.centroid
    else:
        raise TypeError(f"Expected Polygon or MultiPolygon, got {type(main_polygon_deg)}")

    utm_zone = int((centroid_deg.x + 180) / 6) + 1
    hemisphere_code = 6 if centroid_deg.y >= 0 else 7
    crs_proj = f"EPSG:32{hemisphere_code}{utm_zone:02d}" 
    # print(f"Debug (generate_path): Using projected CRS {crs_proj} based on input centroid")

    transformer_to_proj = pyproj.Transformer.from_crs(crs_deg, crs_proj, always_xy=True)
    transformer_to_deg = pyproj.Transformer.from_crs(crs_proj, crs_deg, always_xy=True)

    main_geometry_proj = transform(transformer_to_proj.transform, main_polygon_deg)
    
    # --- Apply Negative Buffer for Insetting based on safety_distance --- 
    main_geometry_proj_for_path = main_geometry_proj # Default to original
    if safety_distance > TOLERANCE: # Only buffer if safety_distance is positive
        inset_distance = -abs(safety_distance)
        print(f"Debug (generate_path): Applying inset buffer: {inset_distance:.4f}m")
        try:
            main_geometry_proj_inset = main_geometry_proj.buffer(inset_distance, join_style=2) # MITRE join style
            # Check if buffering resulted in empty or invalid geometry
            if main_geometry_proj_inset.is_empty or not main_geometry_proj_inset.is_valid:
                print(f"Warning: Negative buffering by {abs(inset_distance):.4f}m resulted in empty/invalid geometry (polygon might be too thin). Using original polygon boundary.")
                # Keep main_geometry_proj_for_path as the original
            else:
                print(f"Debug (generate_path): Using inset geometry for path generation.") 
                main_geometry_proj_for_path = main_geometry_proj_inset
                # Optional: Check area reduction
                # area_orig = main_geometry_proj.area
                # area_inset = main_geometry_proj_for_path.area
                # print(f"Debug (generate_path): Area reduced from {area_orig:.2f} to {area_inset:.2f} sq meters.")
        except Exception as e:
            print(f"Warning: Error applying negative buffer: {e}. Using original polygon boundary.")
            # Keep main_geometry_proj_for_path as the original
    else:
        print("Debug (generate_path): Safety distance is zero or negative, no inset buffer applied.")

    # --- Use the (potentially inset) geometry for subsequent steps --- 
    bounds_proj_for_width = main_geometry_proj_for_path.bounds # Use bounds of inset polygon for width/centroid?
    width_proj = bounds_proj_for_width[2] - bounds_proj_for_width[0] 
    centroid_proj = main_geometry_proj_for_path.centroid # Use centroid of inset polygon for rotation center

    # --- Rotation Setup --- 
    math_angle_deg = (90.0 - angle_deg) % 360.0 

    # --- Rotate Polygon for Horizontal Scan --- 
    # Rotate the inset geometry
    geometry_rotated = rotate(main_geometry_proj_for_path, -math_angle_deg, origin=centroid_proj)
    
    # --- Generate Path on Rotated Polygon --- 
    bounds_rot = geometry_rotated.bounds
    min_x_rot, min_y_rot, max_x_rot, max_y_rot = bounds_rot

    height_rot = max_y_rot - min_y_rot
    separation_meters = abs(separation_meters) if separation_meters > 1e-6 else 0.1 

    clipped_lines_rot = []
    y_coords_rot = np.linspace(min_y_rot + separation_meters / 2, max_y_rot - separation_meters / 2, num=max(1, int(height_rot / separation_meters)))
    if not y_coords_rot.size: # Handle case where height < separation
        y_coords_rot = np.array([(min_y_rot + max_y_rot) / 2])
        

    for current_y_rot in y_coords_rot:
        scan_line_rot = LineString([(min_x_rot - width_proj*1.1, current_y_rot), 
                                    (max_x_rot + width_proj*1.1, current_y_rot)])

        clipped_geom_rot = geometry_rotated.buffer(0).intersection(scan_line_rot) 
        if clipped_geom_rot.is_empty:
            continue

        segments_rot = []
        if isinstance(clipped_geom_rot, LineString):
            if clipped_geom_rot.length > TOLERANCE:
                segments_rot = [clipped_geom_rot]
        elif isinstance(clipped_geom_rot, MultiLineString):
            segments_rot = [s for s in clipped_geom_rot.geoms if s.length > TOLERANCE]
        
        if segments_rot:
             segments_rot.sort(key=lambda s: s.coords[0][0]) 
             clipped_lines_rot.append(segments_rot)
             
    if not clipped_lines_rot:
         return LineString(), centroid_proj, transformer_to_proj, transformer_to_deg

    # --- Assemble Path in Rotated Coordinates --- 
    final_path_coords_rot = []
    direction = True 
    last_processed_endpoint_coord_rot = None

    for segments_on_line_rot in clipped_lines_rot:
        if not segments_on_line_rot:
            continue
        
        connection_coord_rot = segments_on_line_rot[0].coords[0] if direction else segments_on_line_rot[-1].coords[-1]
        current_endpoint_coord_rot = segments_on_line_rot[-1].coords[-1] if direction else segments_on_line_rot[0].coords[0]

        if last_processed_endpoint_coord_rot:
            if Point(last_processed_endpoint_coord_rot).distance(Point(connection_coord_rot)) > TOLERANCE:
                final_path_coords_rot.append(last_processed_endpoint_coord_rot)
                final_path_coords_rot.append(connection_coord_rot)
            elif not final_path_coords_rot or Point(final_path_coords_rot[-1]).distance(Point(connection_coord_rot)) > TOLERANCE:
                final_path_coords_rot.append(connection_coord_rot)

        elif not final_path_coords_rot: 
             final_path_coords_rot.append(connection_coord_rot)

        coords_this_line_rot = []
        if direction: 
            for k, segment in enumerate(segments_on_line_rot):
                coords = list(segment.coords)
                if not coords_this_line_rot or Point(coords_this_line_rot[-1]).distance(Point(coords[0])) > TOLERANCE:
                     coords_this_line_rot.append(coords[0])
                coords_this_line_rot.extend(coords[1:])
        else: 
            for k, segment in enumerate(reversed(segments_on_line_rot)):
                coords = list(segment.coords)[::-1] 
                if not coords_this_line_rot or Point(coords_this_line_rot[-1]).distance(Point(coords[0])) > TOLERANCE:
                     coords_this_line_rot.append(coords[0])
                coords_this_line_rot.extend(coords[1:])
                
        if final_path_coords_rot and Point(final_path_coords_rot[-1]).distance(Point(coords_this_line_rot[0])) < TOLERANCE:
             final_path_coords_rot.extend(coords_this_line_rot[1:])
        else:
             final_path_coords_rot.extend(coords_this_line_rot)
             
        last_processed_endpoint_coord_rot = current_endpoint_coord_rot
        direction = not direction

    if last_processed_endpoint_coord_rot and (not final_path_coords_rot or Point(final_path_coords_rot[-1]).distance(Point(last_processed_endpoint_coord_rot)) > TOLERANCE):
         final_path_coords_rot.append(last_processed_endpoint_coord_rot)

    if len(final_path_coords_rot) < 2:
        return LineString(), centroid_proj, transformer_to_proj, transformer_to_deg
    
    # --- Clean path in rotated coordinates first ---
    cleaned_coords_rot = [final_path_coords_rot[0]]
    for i in range(1, len(final_path_coords_rot)):
        if Point(final_path_coords_rot[i]).distance(Point(cleaned_coords_rot[-1])) > TOLERANCE:
            cleaned_coords_rot.append(final_path_coords_rot[i])
            
    if len(cleaned_coords_rot) < 2:
         return LineString(), centroid_proj, transformer_to_proj, transformer_to_deg
         
    final_path_rot = LineString(cleaned_coords_rot)
    final_path_rot = final_path_rot.simplify(TOLERANCE * 10, preserve_topology=True)
    if final_path_rot.is_empty or len(final_path_rot.coords) < 2:
         return LineString(), centroid_proj, transformer_to_proj, transformer_to_deg
         
    # --- Rotate Path Back to Original Orientation (Projected) --- 
    final_path_proj = rotate(final_path_rot, math_angle_deg, origin=centroid_proj)
    
    # --- Final Cleanup in Projected Coordinates --- 
    if final_path_proj.is_empty or len(final_path_proj.coords) < 2:
         return LineString(), centroid_proj, transformer_to_proj, transformer_to_deg
         
    cleaned_coords_proj = [final_path_proj.coords[0]]
    for i in range(1, len(final_path_proj.coords)):
        if Point(final_path_proj.coords[i]).distance(Point(cleaned_coords_proj[-1])) > TOLERANCE:
             cleaned_coords_proj.append(final_path_proj.coords[i])
             
    if len(cleaned_coords_proj) < 2:
         return LineString(), centroid_proj, transformer_to_proj, transformer_to_deg

    final_path_clean_proj = LineString(cleaned_coords_proj)

    return final_path_clean_proj, centroid_proj, transformer_to_proj, transformer_to_deg

def split_path_by_obstacles(base_path_proj, obstacles_proj):
    """
    Splits a base path LineString (PROJECTED) by subtracting obstacles (PROJECTED).
    Assumes the obstacles_proj passed in have ALREADY been buffered by the desired safety distance.
    Returns a list of LineString tracks (PROJECTED) that are outside obstacles.
    """
    if not base_path_proj.is_valid or base_path_proj.is_empty:
        print("Warning: Base path is invalid or empty for splitting.")

        return [base_path_proj]

    valid_obstacles_proj = [obs for obs in obstacles_proj if obs.is_valid and not obs.is_empty]
    if not valid_obstacles_proj:
        # No valid obstacles provided (maybe originals were invalid or buffer failed)
        # print("No valid projected obstacles to split by.")
        return [base_path_proj]

    # --- Difference Operation (Projected Coords) ---
    # The obstacles are already buffered, so just union them
    try:
         obstacles_union_proj = unary_union(valid_obstacles_proj)
         # Check if union is valid before proceeding
         if not obstacles_union_proj.is_valid:
             print("Warning: Union of obstacles resulted in invalid geometry. Proceeding without splitting.")
             return [base_path_proj]
    except Exception as e:
         print(f"Error unioning obstacles: {e}. Proceeding without splitting.")
         return [base_path_proj]

    try:
        # Perform the difference using the (already buffered) obstacle union
        path_after_difference = base_path_proj.difference(obstacles_union_proj)
    except Exception as e:
        print(f"Error performing difference operation: {e}. Proceeding without splitting.")
        return [base_path_proj]

    # --- Extract Valid LineString Tracks --- 
    tracks_proj = []
    if path_after_difference.is_empty:
        pass # print("Warning: Path is empty after difference operation.") # Removed
    elif isinstance(path_after_difference, LineString):
        if path_after_difference.length > TOLERANCE:
            tracks_proj = [path_after_difference]
    elif isinstance(path_after_difference, MultiLineString):
        tracks_proj = sorted(
            [line for line in path_after_difference.geoms if line.length > TOLERANCE],
            key=lambda line: base_path_proj.project(Point(line.coords[0])) if line.coords else float('inf')
        )
    elif isinstance(path_after_difference, GeometryCollection):
        # print("Info: Difference resulted in GeometryCollection. Extracting and sorting LineStrings.") # Removed
        lines = []
        for geom in path_after_difference.geoms:
            if isinstance(geom, LineString):
                if geom.length > TOLERANCE:
                     lines.append(geom)
            elif isinstance(geom, MultiLineString):
                 lines.extend([line for line in geom.geoms if line.length > TOLERANCE])
        tracks_proj = sorted(
             lines,
             key=lambda line: base_path_proj.project(Point(line.coords[0])) if line.coords else float('inf')
        )
    # else:
        # print(f"Warning: Unexpected geometry type after difference: {type(path_after_difference)}. No tracks generated.") # Removed

    if not tracks_proj:
        # print("Warning: No valid tracks found after splitting by obstacles.") # Removed
        return []

    # --- Clean Tracks (remove duplicate consecutive points) ---
    cleaned_tracks_proj = []
    for track in tracks_proj:
        if not track.coords: continue
        cleaned_coords = [track.coords[0]]
        for i in range(1, len(track.coords)):
            if Point(track.coords[i]).distance(Point(cleaned_coords[-1])) > TOLERANCE:
                cleaned_coords.append(track.coords[i])
        if len(cleaned_coords) >= 2:
            cleaned_tracks_proj.append(LineString(cleaned_coords))
            
    # print(f"Split into {len(cleaned_tracks_proj)} cleaned tracks (projected).") # Removed
    return cleaned_tracks_proj

def _find_intersection_points(path_proj, obstacles_proj):
    """
    Finds points where a LineString intersects obstacle boundaries.
    Inputs path and obstacles must be in the same PROJECTED CRS.
    Returns a list of coordinate tuples (PROJECTED) for intersection points.
    """
    intersection_points_coords = [] 
    if not path_proj.is_valid or path_proj.is_empty or not obstacles_proj:
        return intersection_points_coords 

    valid_obstacles_proj = [obs for obs in obstacles_proj 
                           if isinstance(obs, (Polygon, MultiPolygon)) and obs.is_valid and not obs.is_empty]
    if not valid_obstacles_proj: 
        # print("No valid projected obstacles provided for intersection check.") # Removed
        return intersection_points_coords
    
    # print(f"Finding intersections between path and {len(valid_obstacles_proj)} projected obstacles...") # Removed

    try:
        obstacle_boundaries = unary_union([obs.boundary for obs in valid_obstacles_proj])
    except Exception as e:
        print(f"Error getting obstacle boundaries: {e}. Cannot find intersections.")
        return []

    if obstacle_boundaries.is_empty:
        # print("Obstacle boundaries are empty. No intersections possible.") # Removed
        return []

    intersection = path_proj.intersection(obstacle_boundaries)

    if intersection.is_empty:
        # print("No intersection points found.") # Removed
        return []
        
    points_to_add = []
    if isinstance(intersection, Point):
        points_to_add.append((intersection.x, intersection.y))
    elif isinstance(intersection, MultiPoint):
        for point in intersection.geoms:
             points_to_add.append((point.x, point.y))
    elif isinstance(intersection, (LineString, MultiLineString, GeometryCollection)):
        # If intersection is linear/complex, extract representative points (e.g., start/end of lines)
        # This handles cases where the path runs *along* a boundary
        # print(f"Warning: Intersection resulted in complex geometry ({type(intersection)}). Extracting points from components.") # Removed
        if isinstance(intersection, GeometryCollection):
            geoms = intersection.geoms
        else: # LineString or MultiLineString
            geoms = getattr(intersection, 'geoms', [intersection]) # Handle single LineString
            
        for geom in geoms:
            if isinstance(geom, Point):
                 points_to_add.append((geom.x, geom.y))
            elif isinstance(geom, LineString) and geom.coords:
                 # Add start and end points of linear intersections
                 points_to_add.append(geom.coords[0])
                 points_to_add.append(geom.coords[-1])
                 # Optionally add interpolated points if needed, but start/end usually suffice
            elif isinstance(geom, MultiPoint):
                 for point in geom.geoms:
                      points_to_add.append((point.x, point.y))

    # --- Filter out points that are very close to original path vertices ---
    # This avoids adding points that were already part of the path definition
    original_path_points = { (round(p[0], 7), round(p[1], 7)) for p in path_proj.coords }
    
    for pt_coord in points_to_add:
         intersection_points_coords.append(pt_coord)
            
   
    # Remove duplicates from the collected points using rounding
    unique_intersection_points = []
    seen_coords = set()
    for pt_coord in intersection_points_coords:
        rounded_coord = (round(pt_coord[0], 7), round(pt_coord[1], 7))
        if rounded_coord not in seen_coords:
            # Further check: ensure point actually lies on *an* obstacle boundary
            # Increase tolerance slightly for this check
            is_on_boundary = False
            for obs_bdy in getattr(obstacle_boundaries, 'geoms', [obstacle_boundaries]):
                 if obs_bdy.distance(Point(pt_coord)) < (TOLERANCE * 10):
                      is_on_boundary = True
                      break
            if is_on_boundary:
                 unique_intersection_points.append(pt_coord)
                 seen_coords.add(rounded_coord)
            # else:
            #      print(f"Debug: Point {pt_coord} from intersection not close enough to final boundary, filtering out.") # Removed
           
    # print(f"Found {len(unique_intersection_points)} unique intersection points (projected).") # Removed
    # RETURN PROJECTED POINTS
    return unique_intersection_points
