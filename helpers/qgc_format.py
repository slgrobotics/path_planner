
from shapely.geometry import Polygon, MultiPolygon, Point
from lxml import etree
import simplekml
import zipfile # Needed for KMZ
import json

TOLERANCE = 1e-9  # Tolerance for floating point comparisons

#
# QGroundControl .plan file format handling
#

def read_qgc_plan_polygons(plan_file, num_segments=8):
    """
    Read polygons from a QGroundControl .plan file.
    Returns a list of tuples, where each tuple is (name, polygon_geometry).
    The polygons are in WGS84 (lat, lon) format - converted to (lon, lat) for Shapely.
    """
    print(f"Reading QGC Plan file: {plan_file}")
    
    with open(plan_file, 'r', encoding='utf-8') as f:
        data = json.load(f)

    if not isinstance(data, dict):
        raise ValueError("QGC plan file must contain a top-level JSON object.")

    if data.get("fileType") != "Plan":
        raise ValueError("QGC plan file must have fileType='Plan'.")

    geo_fence = data.get("geoFence")
    if not isinstance(geo_fence, dict):
        raise ValueError("QGC plan file does not contain a valid 'geoFence' object.")

    all_polygons_data = []
    
    # Read inclusion polygons (these become targets)
    for i, item in enumerate(geo_fence.get("polygons", [])):
        if not isinstance(item, dict):
            continue

        raw_polygon = item.get("polygon")
        inclusion = bool(item.get("inclusion", False))

        if not isinstance(raw_polygon, list):
            continue

        points = []
        for pt in raw_polygon:
            if not isinstance(pt, list) or len(pt) < 2:
                continue
            # QGC plan stores as [lat, lon], Shapely needs (lon, lat)
            lat, lon = float(pt[0]), float(pt[1])
            points.append((lon, lat))

        if len(points) < 3:
            continue

        polygon = Polygon(points)
        if polygon.is_valid:
            zone_type = "inclusion" if inclusion else "exclusion"
            name = f"polygon_{i}_{zone_type}"
            all_polygons_data.append((name, polygon))
    
    # Read circles as polygons (approximated)
    for i, item in enumerate(geo_fence.get("circles", [])):
        if not isinstance(item, dict):
            continue

        raw_circle = item.get("circle")
        inclusion = bool(item.get("inclusion", False))

        if not isinstance(raw_circle, dict):
            continue

        center = raw_circle.get("center")
        radius = raw_circle.get("radius")

        if not isinstance(center, list) or len(center) < 2:
            continue

        lat, lon = float(center[0]), float(center[1])
        radius_m = float(radius)
        
        # Approximate circle as a polygon with configurable segments
        from math import radians, sin, cos
        circle_points = []
        for j in range(num_segments):
            angle = 2 * 3.14159 * j / num_segments
            # Approximate: 1 degree at equator ~ 111km
            dlat = (radius_m / 111320) * cos(radians(lat))
            dlon = (radius_m / 111320)
            circle_points.append((lon + dlon * sin(angle), lat + dlat * cos(angle)))
        
        circle_polygon = Polygon(circle_points)
        if circle_polygon.is_valid:
            zone_type = "inclusion" if inclusion else "exclusion"
            name = f"circle_{i}_{zone_type}"
            all_polygons_data.append((name, circle_polygon))

    print(f"Read {len(all_polygons_data)} total polygons from QGC Plan file.")
    return all_polygons_data


def save_path_to_qgc_plan(path_points, output_file, reverse_path=False, altitude_m=20.0):
    """
    Save a list of (lon, lat) points to a QGroundControl .plan file.
    
    Args:
        path_points: List of (lon, lat) tuples or a Shapely LineString.
        output_file: Path to the output .plan file.
        reverse_path: If True, reverse the order of points.
        altitude_m: Altitude in meters for waypoints. Default is 20.0.
    """
    from shapely.geometry import LineString
    
    # Extract coordinates from various input types
    if path_points is None:
        print("No path points provided. .plan file not saved.")
        return False
    
    # Handle LineString input
    if isinstance(path_points, LineString):
        coords_list = list(path_points.coords)
    elif isinstance(path_points, (list, tuple)):
        coords_list = list(path_points)
    else:
        print(f"Invalid path_points type: {type(path_points)}")
        return False
    
    if not coords_list or len(coords_list) < 2:
        print("Path has fewer than 2 points. .plan file not saved.")
        return False
    
    if reverse_path:
        print("Reversing final path points before saving to .plan.")
        coords_list = list(reversed(coords_list))
    
    # Build mission items (waypoints)
    items = []
    do_jump_id = 1
    
    # First item: QGC visual waypoint (command 530) - marks start of survey
    items.append({
        "autoContinue": True,
        "command": 530,
        "doJumpId": do_jump_id,
        "frame": 2,
        "params": [0, 2, None, None, None, None, None],
        "type": "SimpleItem"
    })
    do_jump_id += 1
    
    # Home position (takeoff location) - use first waypoint position
    first_lon, first_lat = coords_list[0]
    items.append({
        "AMSLAltAboveTerrain": altitude_m,
        "Altitude": altitude_m,
        "AltitudeMode": 1,
        "autoContinue": True,
        "command": 16,
        "doJumpId": do_jump_id,
        "frame": 3,
        "params": [0, 0, 0, None, first_lat, first_lon, altitude_m],
        "type": "SimpleItem"
    })
    do_jump_id += 1
    
    # Waypoints for the path
    for lon, lat in coords_list:
        items.append({
            "autoContinue": True,
            "command": 16,
            "doJumpId": do_jump_id,
            "frame": 3,
            "params": [0, 0, 0, None, lat, lon, altitude_m],
            "type": "SimpleItem"
        })
        do_jump_id += 1
    
    # Build the plan JSON
    plan_data = {
        "fileType": "Plan",
        "geoFence": {
            "circles": [],
            "polygons": [],
            "version": 2
        },
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": 1.3,
            "firmwareType": 12,
            "globalPlanAltitudeMode": 1,
            "hoverSpeed": 5,
            "items": items,
            "plannedHomePosition": [None, None, None],
            "vehicleType": 10,
            "version": 2
        },
        "rallyPoints": {
            "points": [],
            "version": 2
        },
        "version": 1
    }
    
    # Save to file
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(plan_data, f, indent=4)
    
    return True

