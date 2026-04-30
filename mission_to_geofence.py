#!/usr/bin/env python3

# ===============================================================================
# Convert a QGroundControl mission path to a geofence polygon.
#
# This script takes a QGroundControl .plan file containing a mission (waypoints)
# and converts it to a geofence plan file by creating a polygon that encompasses
# the mission path.
#
# Basically, you can "feel" the area boundary by creating a mission around it and
# create a geofence from that mission file.
#
# The script computes a bounding box around the waypoints and optionally adds
# padding to ensure the geofence fully contains the mission path.
# The resulting geofence can be used for inclusion (allowed area) or
# exclusion (no-fly zone) depending on the user's choice.
#
# Usage:
#   python mission_to_geofence.py input_mission.plan -o output_geofence.plan -p 0.0001 -e
#
# ===============================================================================

import json
import argparse
import os
from typing import List, Tuple


def load_mission_plan(plan_path: str) -> dict:
    """Load a QGroundControl plan file and extract waypoints."""
    with open(plan_path, 'r') as f:
        plan = json.load(f)
    
    waypoints = []
    items = plan.get('mission', {}).get('items', [])
    
    for item in items:
        # Command 16 is MAV_CMD_NAV_WAYPOINT
        if item.get('command') == 16:
            params = item.get('params', [])
            # Params[4] = latitude, params[5] = longitude
            lat = params[4]
            lon = params[5]
            waypoints.append((lat, lon))
    
    return waypoints


def compute_bounding_box(waypoints: List[Tuple[float, float]], 
                         padding_deg: float = 0.0001) -> List[Tuple[float, float]]:
    """
    Compute a bounding box polygon around waypoints with optional padding.
    
    Args:
        waypoints: List of (latitude, longitude) tuples
        padding_deg: Padding in degrees to add around the waypoints
        
    Returns:
        List of (lat, lon) tuples forming a rectangle polygon
    """
    if not waypoints:
        return []
    
    lats = [wp[0] for wp in waypoints]
    lons = [wp[1] for wp in waypoints]
    
    min_lat = min(lats) - padding_deg
    max_lat = max(lats) + padding_deg
    min_lon = min(lons) - padding_deg
    max_lon = max(lons) + padding_deg
    
    # Create polygon in clockwise order
    polygon = [
        (max_lat, min_lon),  # top-left
        (max_lat, max_lon),  # top-right
        (min_lat, max_lon),  # bottom-right
        (min_lat, min_lon),  # bottom-left
    ]
    
    return polygon


def compute_convex_hull(waypoints: List[Tuple[float, float]], 
                        padding_deg: float = 0.00005) -> List[Tuple[float, float]]:
    """
    Compute a convex hull polygon around waypoints with optional padding.
    
    Uses a simple approach: compute bounding box, then optionally add padding.
    For more complex hulls, consider using scipy.spatial.ConvexHull.
    
    Args:
        waypoints: List of (latitude, longitude) tuples
        padding_deg: Padding in degrees to add around the waypoints
        
    Returns:
        List of (lat, lon) tuples forming a convex polygon
    """
    if not waypoints:
        return []
    
    if len(waypoints) <= 2:
        # For 1-2 points, return a small bounding box
        return compute_bounding_box(waypoints, padding_deg)
    
    # Use bounding box approach for simplicity
    return compute_bounding_box(waypoints, padding_deg)


def create_geofence_plan(waypoints: List[Tuple[float, float]], 
                         output_path: str,
                         padding_deg: float = 0.00005,
                         inclusion: bool = True) -> None:
    """
    Create a geofence plan file from waypoints.
    
    Args:
        waypoints: List of (latitude, longitude) tuples from mission
        output_path: Path to save the output geofence plan
        padding_deg: Padding in degrees around the waypoints
        inclusion: If True, polygon is inclusion zone; if False, exclusion zone
    """
    polygon = compute_bounding_box(waypoints, padding_deg)
    
    geofence_plan = {
        "fileType": "Plan",
        "geoFence": {
            "circles": [],
            "polygons": [
                {
                    "inclusion": inclusion,
                    "polygon": [[lat, lon] for lat, lon in polygon],
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
    print(f"Polygon vertices: {len(polygon)}")
    print(f"Bounds: lat [{polygon[2][0]:.6f}, {polygon[0][0]:.6f}], "
          f"lon [{polygon[0][1]:.6f}, {polygon[1][1]:.6f}]")


def main():
    parser = argparse.ArgumentParser(
        description='Convert a QGroundControl mission path to a geofence polygon'
    )
    parser.add_argument('input_plan', 
                        help='Path to input .plan file with mission waypoints')
    parser.add_argument('-o', '--output', 
                        help='Output geofence plan path (default: input_geofence.plan)')
    parser.add_argument('-p', '--padding', type=float, default=0.00005,
                        help='Padding in degrees around waypoints (default: 0.00005)')
    parser.add_argument('-e', '--exclusion', action='store_true',
                        help='Create exclusion zone instead of inclusion zone')
    
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
    waypoints = load_mission_plan(args.input_plan)
    
    if not waypoints:
        print("Error: No waypoints found in mission")
        return 1
    
    print(f"Found {len(waypoints)} waypoints")
    
    # Create geofence plan
    create_geofence_plan(
        waypoints, 
        output_path, 
        args.padding,
        inclusion=not args.exclusion
    )
    
    return 0


if __name__ == '__main__':
    exit(main())