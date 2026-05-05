#!/usr/bin/env python3

# ===============================================================================
#
# Combine multiple QGroundControl .plan files into a single .plan output.
#
# This script merges mission items, geofence polygons, geofence circles, and rally
# points from multiple QGroundControl plan files into a single combined plan.
#
# Usage:
#   python3 combine_plans.py output_combined.plan input1.plan input2.plan ...
#
# Created by: VS Code Copilot (Free, Claude Haiku 4.5)
# Date: 2026-05-05
# ===============================================================================

import argparse
import json
import os
from typing import Any, Dict, List, Optional, Sequence, Tuple


def load_qgc_plan(plan_path: str) -> Dict[str, Any]:
    with open(plan_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    if not isinstance(data, dict):
        raise ValueError(f'Plan file {plan_path} does not contain a JSON object.')

    if data.get('fileType') != 'Plan':
        raise ValueError(f'Plan file {plan_path} is not a QGroundControl Plan file.')

    return data


def normalized_list(value: Optional[Sequence[Any]]) -> List[Any]:
    if value is None:
        return []
    return list(value)


def choose_value(values: Sequence[Any], default: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return default


def combine_plans(plan_files: Sequence[str]) -> Dict[str, Any]:
    combined_geo_polygons: List[Any] = []
    combined_geo_circles: List[Any] = []
    combined_rally_points: List[Any] = []
    combined_items: List[Any] = []

    home_position: Optional[List[Any]] = None
    cruise_speed: Optional[float] = None
    hover_speed: Optional[float] = None
    firmware_type: Optional[int] = None
    global_plan_altitude_mode: Optional[int] = None
    vehicle_type: Optional[int] = None

    ground_station = 'QGroundControl'
    plan_version = 1

    for plan_path in plan_files:
        plan = load_qgc_plan(plan_path)
        geo_fence = plan.get('geoFence', {})
        mission = plan.get('mission', {})
        rally_points = plan.get('rallyPoints', {})

        combined_geo_polygons.extend(normalized_list(geo_fence.get('polygons')))
        combined_geo_circles.extend(normalized_list(geo_fence.get('circles')))
        combined_rally_points.extend(normalized_list(rally_points.get('points')))

        items = normalized_list(mission.get('items'))
        combined_items.extend(items)

        if home_position is None:
            candidate_home = mission.get('plannedHomePosition')
            if isinstance(candidate_home, list) and any(x is not None for x in candidate_home):
                home_position = candidate_home

        if cruise_speed is None:
            cruise_speed = mission.get('cruiseSpeed')
        if hover_speed is None:
            hover_speed = mission.get('hoverSpeed')
        if firmware_type is None:
            firmware_type = mission.get('firmwareType')
        if global_plan_altitude_mode is None:
            global_plan_altitude_mode = mission.get('globalPlanAltitudeMode')
        if vehicle_type is None:
            vehicle_type = mission.get('vehicleType')

        if plan_version is None:
            plan_version = plan.get('version', plan_version)
        if plan.get('groundStation'):
            ground_station = plan.get('groundStation')

    # Renumber mission item doJumpId values sequentially
    for index, item in enumerate(combined_items, start=1):
        if isinstance(item, dict):
            item['doJumpId'] = index

    if home_position is None and combined_items:
        first_item = combined_items[0]
        if isinstance(first_item, dict):
            params = first_item.get('params')
            if isinstance(params, list) and len(params) >= 6:
                home_lat = params[4]
                home_lon = params[5]
                if home_lat is not None and home_lon is not None:
                    home_position = [home_lat, home_lon, None]

    combined_plan: Dict[str, Any] = {
        'fileType': 'Plan',
        'geoFence': {
            'circles': combined_geo_circles,
            'polygons': combined_geo_polygons,
            'version': 2,
        },
        'groundStation': ground_station,
        'mission': {
            'cruiseSpeed': float(cruise_speed) if cruise_speed is not None else 1.3,
            'firmwareType': int(firmware_type) if firmware_type is not None else 12,
            'globalPlanAltitudeMode': int(global_plan_altitude_mode) if global_plan_altitude_mode is not None else 1,
            'hoverSpeed': float(hover_speed) if hover_speed is not None else 5,
            'items': combined_items,
            'plannedHomePosition': home_position if home_position is not None else [None, None, None],
            'vehicleType': int(vehicle_type) if vehicle_type is not None else 10,
            'version': 2,
        },
        'rallyPoints': {
            'points': combined_rally_points,
            'version': 2,
        },
        'version': int(plan_version) if plan_version is not None else 1,
    }

    return combined_plan


def save_combined_plan(combined_plan: Dict[str, Any], output_path: str) -> None:
    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(combined_plan, f, indent=4)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Combine multiple QGroundControl .plan files into one combined .plan file.'
    )
    parser.add_argument('-o', '--output', required=True, help='Path to the combined output .plan file')
    parser.add_argument('input_plans', nargs='+', help='Input QGroundControl .plan files to combine')
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    for plan_path in args.input_plans:
        if not os.path.exists(plan_path):
            print(f'Error: Input file not found: {plan_path}')
            return 1

    combined_plan = combine_plans(args.input_plans)
    save_combined_plan(combined_plan, args.output)
    print(f'Combined {len(args.input_plans)} plan files into: {args.output}')
    print(f'  Mission items: {len(combined_plan["mission"]["items"])}')
    print(f'  Geofence polygons: {len(combined_plan["geoFence"]["polygons"])}')
    print(f'  Geofence circles: {len(combined_plan["geoFence"]["circles"])}')
    print(f'  Rally points: {len(combined_plan["rallyPoints"]["points"])}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
