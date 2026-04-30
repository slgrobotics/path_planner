#!/usr/bin/env python3

# ===============================================================================
# Convert a QGroundControl mission path to a geofence polygon.
#
# This script takes a QGroundControl .plan file containing a mission (waypoints)
# and converts it to a geofence plan file.
# The script converts mission waypoints to a geofence polygon corners.
# The resulting geofence can be used for inclusion (allowed area) or
# exclusion (no-fly zone) depending on the user's choice ("-e" argument).
#
# Basically, you can "feel" the area boundary by creating a mission around it
# and create a geofence from that mission file.
#
# Usage:
#   python mission_to_geofence.py input_mission.plan -o output_geofence.plan [-e]
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


def create_geofence_plan(waypoints: List[Tuple[float, float]], 
                         output_path: str,
                         inclusion: bool = True) -> None:
    """
    Create a geofence plan file from waypoints.
    
    Uses the mission waypoints directly as geofence polygon vertices.
    
    Args:
        waypoints: List of (latitude, longitude) tuples from mission
        output_path: Path to save the output geofence plan
        inclusion: If True, polygon is inclusion zone; if False, exclusion zone
    """
    if not waypoints:
        print("Error: No waypoints to convert")
        return
    
    # Use waypoints directly as polygon vertices (no bounding box, no padding)
    polygon = [[lat, lon] for lat, lon in waypoints]
    
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
        inclusion=not args.exclusion
    )
    
    return 0


if __name__ == '__main__':
    exit(main())