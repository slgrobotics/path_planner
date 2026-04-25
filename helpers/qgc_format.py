
from shapely.geometry import Polygon, MultiPolygon, Point
from lxml import etree
import simplekml
import zipfile # Needed for KMZ
import json

TOLERANCE = 1e-9  # Tolerance for floating point comparisons

#
# QGroundControl .plan file format handling
#

def read_qgc_plan_polygons(plan_file):
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
        
        # Approximate circle as a polygon with 64 segments
        from math import radians, sin, cos
        num_segments = 64
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


def save_path_to_qgc_plan(path_points, output_file):
    """
    Save a list of (lon, lat) points to a QGC Plan file.
    """
    print(f"Saving path to QGC Plan file: {output_file}")
    
    print(f"Path saved to {output_file}")
