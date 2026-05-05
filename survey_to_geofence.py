#!/usr/bin/env python3

# ===============================================================================
# Convert a QGroundControl survey plan to a geofence polygon.
#
# This script takes a QGroundControl .plan file containing a survey complex item
# and converts the survey polygon into a geofence inclusion zone.
#
# Usage:
#   python3 survey_to_geofence.py input_survey.plan [-o output_geofence.plan]
#
# Created by: VS Code Copilot (Free, Claude Haiku 4.5)
# Date: 2026-05-05
# ===============================================================================

import argparse
import json
import os
from typing import Any, Dict, List, Optional


def load_qgc_plan(plan_path: str) -> Dict[str, Any]:
    """Load a QGroundControl .plan JSON file."""
    with open(plan_path, 'r', encoding='utf-8') as f:
        plan = json.load(f)

    if not isinstance(plan, dict) or plan.get('fileType') != 'Plan':
        raise ValueError('Input file is not a valid QGroundControl .plan file.')

    return plan


def extract_survey_polygons(plan: Dict[str, Any]) -> List[List[List[float]]]:
    """Extract survey polygons from QGC mission complex items."""
    polygons: List[List[List[float]]] = []
    items = plan.get('mission', {}).get('items', [])

    def recurse_item(item: Dict[str, Any]) -> None:
        if not isinstance(item, dict):
            return

        if item.get('complexItemType') == 'survey':
            survey_polygon = item.get('polygon')
            if isinstance(survey_polygon, list) and len(survey_polygon) >= 3:
                # Ensure the polygon is a list of [lat, lon] pairs
                cleaned = []
                for point in survey_polygon:
                    if (isinstance(point, list) or isinstance(point, tuple)) and len(point) >= 2:
                        lat = float(point[0])
                        lon = float(point[1])
                        cleaned.append([lat, lon])
                if len(cleaned) >= 3:
                    polygons.append(cleaned)

            # Some survey items nest the actual complex item inside TransectStyleComplexItem.
            nested = item.get('TransectStyleComplexItem')
            if isinstance(nested, dict):
                nested_polygon = nested.get('polygon')
                if isinstance(nested_polygon, list) and len(nested_polygon) >= 3:
                    cleaned = []
                    for point in nested_polygon:
                        if (isinstance(point, list) or isinstance(point, tuple)) and len(point) >= 2:
                            lat = float(point[0])
                            lon = float(point[1])
                            cleaned.append([lat, lon])
                    if len(cleaned) >= 3:
                        polygons.append(cleaned)

        # Recurse into common nested structures
        if 'Items' in item and isinstance(item['Items'], list):
            for child in item['Items']:
                recurse_item(child)

        if 'TransectStyleComplexItem' in item and isinstance(item['TransectStyleComplexItem'], dict):
            nested_item = item['TransectStyleComplexItem']
            if 'Items' in nested_item and isinstance(nested_item['Items'], list):
                for child in nested_item['Items']:
                    recurse_item(child)

    for entry in items:
        recurse_item(entry)

    return polygons


def create_geofence_plan(polygons: List[List[List[float]]], output_path: str) -> None:
    """Save a QGroundControl geofence plan from survey polygons."""
    geofence_polygons = []
    for polygon in polygons:
        geofence_polygons.append({
            'inclusion': True,
            'polygon': polygon,
            'version': 1
        })

    geofence_plan = {
        'fileType': 'Plan',
        'geoFence': {
            'circles': [],
            'polygons': geofence_polygons,
            'version': 2
        },
        'groundStation': 'QGroundControl',
        'mission': {
            'cruiseSpeed': 15,
            'firmwareType': 12,
            'globalPlanAltitudeMode': 1,
            'hoverSpeed': 5,
            'items': [],
            'plannedHomePosition': [None, None, None],
            'vehicleType': 10,
            'version': 2
        },
        'rallyPoints': {
            'points': [],
            'version': 2
        },
        'version': 1
    }

    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(geofence_plan, f, indent=4)

    print(f'Geofence created: {output_path}')
    print(f'Polygons saved: {len(geofence_polygons)}')
    for index, polygon in enumerate(geofence_polygons, start=1):
        print(f'  Polygon {index} vertices: {len(polygon["polygon"])}')


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Convert a QGroundControl survey .plan file to a geofence plan.'
    )
    parser.add_argument('input_plan', help='Path to input .plan file containing survey items')
    parser.add_argument('-o', '--output', help='Output geofence plan path (default: input_survey_geofence.plan)')

    args = parser.parse_args()

    if not os.path.exists(args.input_plan):
        print(f'Error: Input file not found: {args.input_plan}')
        return 1

    output_path = args.output
    if not output_path:
        base, _ = os.path.splitext(args.input_plan)
        output_path = f'{base}_survey_geofence.plan'

    plan = load_qgc_plan(args.input_plan)
    polygons = extract_survey_polygons(plan)

    if not polygons:
        print('Error: No survey polygon found in input plan.')
        return 1

    create_geofence_plan(polygons, output_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
