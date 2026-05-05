#!/usr/bin/env python3

# ===============================================================================
# Convert a QGroundControl geofence polygon to a mission path.
#
# This script takes a QGroundControl .plan file containing a geofence polygon
# and converts it to a mission plan file by creating waypoints at the polygon
# corners.
#
# The resulting mission can be used to "feel" or trace the boundary of a
# geofence area.
#
# Usage:
#   python3 geofence_to_mission.py input_geofence.plan [-o output_mission.plan] [-a ALTITUDE]
#
# Created by: VS Code Copilot (Free, Claude Haiku 4.5)
# Date: 2026-04-28
# ===============================================================================

import json
import argparse
import os
from typing import List, Tuple


def load_geofence_plan(plan_path: str) -> List[Tuple[float, float]]:
    """Load a QGroundControl plan file and extract geofence polygon vertices."""
    with open(plan_path, 'r') as f:
        plan = json.load(f)
    
    polygons = plan.get('geoFence', {}).get('polygons', [])
    
    if not polygons:
        print("Warning: No polygons found in geofence")
        return []
    
    # Get the first polygon's vertices
    polygon = polygons[0].get('polygon', [])
    
    # Extract [lat, lon] pairs
    vertices = [(point[0], point[1]) for point in polygon]
    
    return vertices


def create_mission_plan(vertices: List[Tuple[float, float]], 
                        output_path: str,
                        altitude: float = 20.0,
                        cruise_speed: float = 1.3) -> None:
    """
    Create a mission plan file from geofence polygon vertices.
    
    Args:
        vertices: List of (latitude, longitude) tuples from geofence polygon
        output_path: Path to save the output mission plan
        altitude: Flight altitude in meters
        cruise_speed: Mission cruise speed in m/s
    """
    if not vertices:
        print("Error: No vertices to convert")
        return
    
    # Create mission items from polygon vertices
    items = []
    for i, (lat, lon) in enumerate(vertices, start=1):
        item = {
            "AMSLAltAboveTerrain": altitude,
            "Altitude": altitude,
            "AltitudeMode": 1,
            "autoContinue": True,
            "command": 16,  # MAV_CMD_NAV_WAYPOINT
            "doJumpId": i,
            "frame": 3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT
            "params": [
                0,      # Hold time (seconds)
                0,      # Acceptance radius (m)
                0,      # Pass radius (m)
                None,   # Yaw angle (None = auto)
                lat,
                lon,
                altitude
            ],
            "type": "SimpleItem"
        }
        items.append(item)
    
    # Calculate approximate home position (center of polygon)
    center_lat = sum(v[0] for v in vertices) / len(vertices)
    center_lon = sum(v[1] for v in vertices) / len(vertices)
    
    mission_plan = {
        "fileType": "Plan",
        "geoFence": {
            "circles": [],
            "polygons": [],
            "version": 2
        },
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": cruise_speed,
            "firmwareType": 12,
            "globalPlanAltitudeMode": 1,
            "hoverSpeed": 5,
            "items": items,
            "plannedHomePosition": [
                center_lat,
                center_lon,
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
        json.dump(mission_plan, f, indent=4)
    
    print(f"Mission created: {output_path}")
    print(f"Waypoints: {len(items)}")
    print(f"Altitude: {altitude}m, Cruise speed: {cruise_speed} m/s")


def main():
    parser = argparse.ArgumentParser(
        description='Convert a QGroundControl geofence polygon to a mission path'
    )
    parser.add_argument('input_plan', 
                        help='Path to input .plan file with geofence polygon')
    parser.add_argument('-o', '--output', 
                        help='Output mission plan path (default: input_mission.plan)')
    parser.add_argument('-a', '--altitude', type=float, default=20.0,
                        help='Flight altitude in meters (default: 20)')
    parser.add_argument('-s', '--speed', type=float, default=1.3,
                        help='Cruise speed in m/s (default: 1.3)')
    
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
        output_path = f"{base}_mission.plan"
    
    # Load vertices from geofence plan
    print(f"Loading geofence from: {args.input_plan}")
    vertices = load_geofence_plan(args.input_plan)
    
    if not vertices:
        print("Error: No polygon vertices found in geofence")
        return 1
    
    print(f"Found {len(vertices)} polygon vertices")
    
    # Create mission plan
    create_mission_plan(
        vertices, 
        output_path, 
        args.altitude,
        args.speed
    )
    
    return 0


if __name__ == '__main__':
    exit(main())