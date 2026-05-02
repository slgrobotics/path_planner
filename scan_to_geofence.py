#!/usr/bin/env python3

# ===============================================================================
# Convert a QGroundControl survey mission to a geofence inclusion zone.
#
# This script takes a QGroundControl .plan file containing a survey mission
# and creates a geofence inclusion zone based on the convex hull of the mission
# waypoints. The convex hull represents the approximate outer boundary of the
# surveyed area.
#
# Usage:
#   python3 scan_to_geofence.py input_mission.plan [-o output_geofence.plan]
#
# Created by: VS Code Copilot (Free, Claude Haiku 4.5)
# Date: 20246-05-02
# ===============================================================================

import json
import argparse
import os
from shapely.geometry import Polygon, MultiPoint


def load_mission_waypoints(plan_path: str) -> list:
    """Load a QGroundControl plan file and extract mission waypoints."""
    with open(plan_path, 'r') as f:
        plan = json.load(f)
    
    waypoints = []
    items = plan.get('mission', {}).get('items', [])
    
    def extract_waypoints_from_items(items_list):
        for item in items_list:
            if item.get('command') == 16:  # MAV_CMD_NAV_WAYPOINT
                params = item.get('params', [])
                lat = params[4]
                lon = params[5]
                if lat is not None and lon is not None:
                    waypoints.append((lat, lon))
            elif item.get('type') == 'ComplexItem':
                # Handle nested complex items
                if 'Items' in item:
                    extract_waypoints_from_items(item['Items'])
                elif 'TransectStyleComplexItem' in item and 'Items' in item['TransectStyleComplexItem']:
                    extract_waypoints_from_items(item['TransectStyleComplexItem']['Items'])
    
    extract_waypoints_from_items(items)
    return waypoints


def compute_convex_hull_polygon(waypoints: list) -> Polygon:
    """
    Compute the convex hull of the waypoints and return as a Shapely Polygon.
    
    Args:
        waypoints: List of (latitude, longitude) tuples
        
    Returns:
        Shapely Polygon representing the convex hull
    """
    if len(waypoints) < 3:
        if len(waypoints) == 2:
            # For 2 points, buffer the line to create a thin polygon
            points = MultiPoint(waypoints)
            hull = points.convex_hull
            return hull.buffer(0.00001)  # Small buffer to make polygon
        elif len(waypoints) == 1:
            # For 1 point, create a small square around it
            lat, lon = waypoints[0]
            # Create a small square (1m x 1m approximately)
            size = 0.00001  # Roughly 1m at equator
            coords = [
                (lat - size, lon - size),
                (lat - size, lon + size),
                (lat + size, lon + size),
                (lat + size, lon - size),
                (lat - size, lon - size)  # Close the polygon
            ]
            return Polygon(coords)
        else:
            raise ValueError("Need at least 1 waypoint")
    
    # For 3+ points
    points = MultiPoint(waypoints)
    hull = points.convex_hull
    
    if isinstance(hull, Polygon):
        return hull
    else:
        # If hull is a LineString or Point (e.g., collinear points), buffer to make polygon
        return hull.buffer(0.00001)


def create_geofence_plan(polygon: Polygon, output_path: str) -> None:
    """
    Create a geofence plan file from a Shapely Polygon.
    
    Args:
        polygon: Shapely Polygon representing the geofence area
        output_path: Path to save the output geofence plan
    """
    # Extract coordinates from polygon (Shapely uses (x, y) = (lat, lon) since waypoints are (lat, lon))
    coords = list(polygon.exterior.coords)
    
    # Convert to QGC format: [[lat, lon], ...]
    qgc_polygon = [[lat, lon] for lat, lon in coords]
    
    # Remove the closing coordinate if it's the same as the first
    if qgc_polygon and qgc_polygon[0] == qgc_polygon[-1]:
        qgc_polygon = qgc_polygon[:-1]
    
    geofence_plan = {
        "fileType": "Plan",
        "geoFence": {
            "circles": [],
            "polygons": [
                {
                    "inclusion": True,  # Inclusion zone
                    "polygon": qgc_polygon,
                    "version": 1
                }
            ],
            "version": 2
        },
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": 15,
            "firmwareType": 12,
            "globalPlanAltitudeMode": 1,
            "hoverSpeed": 5,
            "items": [],
            "plannedHomePosition": [
                None,
                None,
                None
            ],
            "vehicleType": 10,
            "version": 2
        },
        "rallyPoints": {
            "points": [],
            "version": 2
        },
        "version": 1
    }
    
    with open(output_path, 'w') as f:
        json.dump(geofence_plan, f, indent=4)
    
    print(f"Geofence created: {output_path}")
    print(f"Polygon vertices: {len(qgc_polygon)}")
    
    # Calculate bounds
    if qgc_polygon:
        lats = [pt[0] for pt in qgc_polygon]
        lons = [pt[1] for pt in qgc_polygon]
        print(f"Bounds: lat [{min(lats):.6f}, {max(lats):.6f}], "
              f"lon [{min(lons):.6f}, {max(lons):.6f}]")


def main():
    parser = argparse.ArgumentParser(
        description='Convert a QGroundControl survey mission to a geofence inclusion zone'
    )
    parser.add_argument('input_plan', 
                        help='Path to input .plan file with survey mission')
    parser.add_argument('-o', '--output', 
                        help='Output geofence plan path (default: input_geofence.plan)')
    
    args = parser.parse_args()
    
    # Validate input file
    if not os.path.exists(args.input_plan):
        print(f"Error: Input file not found: {args.input_plan}")
        return 1
    
    # Determine output path
    if args.output:
        output_path = args.output
    else:
        base, ext = os.path.splitext(args.input_plan)
        output_path = f"{base}_geofence.plan"
    
    # Load waypoints from input plan
    print(f"Loading mission from: {args.input_plan}")
    waypoints = load_mission_waypoints(args.input_plan)
    
    if not waypoints:
        print("Error: No waypoints found in mission")
        return 1
    
    print(f"Found {len(waypoints)} waypoints")
    
    # Compute convex hull polygon
    print("Computing convex hull...")
    hull_polygon = compute_convex_hull_polygon(waypoints)
    
    if not hull_polygon.is_valid:
        print("Error: Computed convex hull is not a valid polygon")
        return 1
    
    # Calculate approximate center
    center = hull_polygon.centroid
    print(f"Approximate center: lat {center.y:.6f}, lon {center.x:.6f}")
    
    # Create geofence plan
    create_geofence_plan(hull_polygon, output_path)
    
    return 0


if __name__ == '__main__':
    exit(main())
