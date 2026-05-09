#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualize mission waypoints and geofence polygons from a QGroundControl .plan file.
Displays the data in a 2D plot with x (longitude) and y (latitude) coordinates.
"""

import sys
import json
import argparse
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MPLPolygon
from matplotlib.lines import Line2D
import numpy as np


def read_plan_file(plan_file):
    """
    Read a QGroundControl .plan file and extract mission and geofence data.
    
    Returns:
        mission_points: List of (lon, lat) tuples for waypoints
        polygons: List of (name, [(lon, lat), ...]) tuples for geofence polygons
        circles: List of (name, center_lat, center_lon, radius_m) tuples for geofence circles
    """
    print(f"Reading plan file: {plan_file}")
    
    with open(plan_file, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    if not isinstance(data, dict) or data.get("fileType") != "Plan":
        raise ValueError("Invalid QGroundControl plan file format")
    
    # Extract mission waypoints
    mission_points = []
    mission = data.get("mission", {})
    items = mission.get("items", [])
    
    for item in items:
        # Command 16 is a NAV_WAYPOINT
        if item.get("command") == 16:
            params = item.get("params", [])
            if len(params) >= 7:
                lat, lon, alt = params[4], params[5], params[6]
                if lat is not None and lon is not None:
                    mission_points.append((lon, lat, alt))
    
    # Extract geofence polygons
    polygons = []
    geo_fence = data.get("geoFence", {})
    
    for i, poly_item in enumerate(geo_fence.get("polygons", [])):
        raw_polygon = poly_item.get("polygon", [])
        inclusion = poly_item.get("inclusion", False)
        zone_type = "inclusion" if inclusion else "exclusion"
        name = f"polygon_{i}_{zone_type}"
        
        points = []
        for pt in raw_polygon:
            if isinstance(pt, list) and len(pt) >= 2:
                lat, lon = float(pt[0]), float(pt[1])
                points.append((lon, lat))
        
        if len(points) >= 3:
            polygons.append((name, points, inclusion))
    
    # Extract geofence circles
    circles = []
    for i, circle_item in enumerate(geo_fence.get("circles", [])):
        raw_circle = circle_item.get("circle", {})
        inclusion = circle_item.get("inclusion", False)
        zone_type = "inclusion" if inclusion else "exclusion"
        name = f"circle_{i}_{zone_type}"
        
        center = raw_circle.get("center", [])
        radius = raw_circle.get("radius", 0)
        
        if isinstance(center, list) and len(center) >= 2:
            lat, lon = float(center[0]), float(center[1])
            circles.append((name, lat, lon, radius, inclusion))
    
    print(f"Found {len(mission_points)} mission waypoints")
    print(f"Found {len(polygons)} geofence polygons")
    print(f"Found {len(circles)} geofence circles")
    
    return mission_points, polygons, circles


def plot_plan(mission_points, polygons, circles):
    """
    Create and display a matplotlib plot of the mission and geofences.
    """
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Plot geofence polygons
    colors_inclusion = ['green', 'lightgreen', 'darkgreen', 'lime', 'forestgreen']
    colors_exclusion = ['red', 'lightcoral', 'darkred', 'salmon', 'crimson']
    
    for i, (name, points, inclusion) in enumerate(polygons):
        color = colors_inclusion[i % len(colors_inclusion)] if inclusion else colors_exclusion[i % len(colors_exclusion)]
        alpha = 0.3 if inclusion else 0.2
        
        polygon = MPLPolygon(points, fill=True, edgecolor='black', facecolor=color, alpha=alpha, linewidth=1.5)
        ax.add_patch(polygon)
        
        # Add label at polygon centroid
        if len(points) > 0:
            centroid_x = np.mean([p[0] for p in points])
            centroid_y = np.mean([p[1] for p in points])
            label_type = "inclusion" if inclusion else "exclusion"
            ax.text(centroid_x, centroid_y, f"{label_type}", fontsize=8, ha='center')
    
    # Plot geofence circles
    for i, (name, lat, lon, radius, inclusion) in enumerate(circles):
        color = colors_inclusion[i % len(colors_inclusion)] if inclusion else colors_exclusion[i % len(colors_exclusion)]
        alpha = 0.3 if inclusion else 0.2
        
        # Approximate circle radius in degrees (rough approximation)
        radius_deg = radius / 111000  # ~111 km per degree
        
        circle = plt.Circle((lon, lat), radius_deg, fill=True, edgecolor='black', facecolor=color, alpha=alpha, linewidth=1.5)
        ax.add_patch(circle)
    
    # Plot mission waypoints and path
    if len(mission_points) > 0:
        lons = [p[0] for p in mission_points]
        lats = [p[1] for p in mission_points]
        alts = [p[2] for p in mission_points]
        
        # Plot path as a line
        ax.plot(lons, lats, 'b-', linewidth=2, label='Mission Path', zorder=5)
        
        # Plot waypoints as points
        ax.scatter(lons, lats, c='blue', s=100, marker='o', zorder=6, edgecolors='darkblue', linewidth=1.5)
        
        # Add waypoint numbers
        for i, (lon, lat, alt) in enumerate(mission_points):
            ax.text(lon, lat, f' {i}', fontsize=9, fontweight='bold', color='blue')
    
    # Set labels and title
    ax.set_xlabel('Longitude (degrees)', fontsize=12)
    ax.set_ylabel('Latitude (degrees)', fontsize=12)
    ax.set_title('Mission Plan Visualization', fontsize=14, fontweight='bold')
    
    # Create legend
    legend_elements = [
        Line2D([0], [0], color='blue', linewidth=2, label='Mission Path'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=8, label='Waypoints', markeredgecolor='darkblue'),
        MPLPolygon([(0, 0)], facecolor='green', alpha=0.3, edgecolor='black', label='Inclusion Zones'),
        MPLPolygon([(0, 0)], facecolor='red', alpha=0.2, edgecolor='black', label='Exclusion Zones'),
    ]
    ax.legend(handles=legend_elements, loc='upper right', fontsize=10)
    
    # Enable grid
    ax.grid(True, alpha=0.3, linestyle='--')
    
    # Equal aspect ratio
    ax.set_aspect('equal', adjustable='box')
    
    # Adjust layout
    plt.tight_layout()
    
    # Show plot
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Visualize mission waypoints and geofence polygons from a QGroundControl .plan file'
    )
    parser.add_argument('plan_file', help='Path to the .plan file to visualize')
    parser.add_argument('--no-show', action='store_true', help='Do not display the plot window')
    
    args = parser.parse_args()
    
    try:
        mission_points, polygons, circles = read_plan_file(args.plan_file)
        
        if not args.no_show:
            plot_plan(mission_points, polygons, circles)
        else:
            print("Plan data loaded successfully (plot window suppressed)")
            
    except FileNotFoundError:
        print(f"Error: File not found: {args.plan_file}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
