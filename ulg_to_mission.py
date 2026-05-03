#!/usr/bin/env python3

# ===============================================================================
# Convert a PX4 ULog (.ulg) GPS track into a QGroundControl mission plan.
#
# This script extracts GPS positions from a PX4 ULog file and converts them
# into a QGroundControl .plan mission file. Points closer than the supplied
# "--step" distance are ignored to reduce waypoint density.
#
# Usage:
#   python3 ulg_to_mission.py input_log.ulg [-o output_mission.plan] [--step 2.0] [-a ALTITUDE] [-s CRUISESPEED]
#
# Created by: GitHub Copilot
# Date: 2026-05-03
# ===============================================================================

import argparse
import math
import os
from typing import List, Optional, Sequence, Tuple

try:
    from pyulog import ULog
except ImportError as exc:
    raise SystemExit('pyulog library is required. Install it with "pip3 install pyulog".') from exc

from helpers.qgc_format import save_path_to_qgc_plan

GPS_SOURCES = [
    ('vehicle_gps_position', 'latitude_deg', 'longitude_deg', 'altitude_msl_m'),
    ('sensor_gps', 'latitude_deg', 'longitude_deg', 'altitude_msl_m'),
    ('vehicle_global_position', 'lat', 'lon', 'alt'),
    ('estimator_global_position', 'lat', 'lon', 'alt'),
    ('home_position', 'lat', 'lon', 'alt'),
]


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Return great-circle distance between two points in meters."""
    r = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    return 2.0 * r * math.asin(math.sqrt(a))


def get_gps_source(ulog: ULog, source_name: Optional[str] = None):
    """Return the first available GPS-related ULog message data object."""
    if source_name:
        for source in GPS_SOURCES:
            if source[0] == source_name:
                return next((d for d in ulog.data_list if d.name == source_name), None), source
        return None, None

    for source in GPS_SOURCES:
        data = next((d for d in ulog.data_list if d.name == source[0]), None)
        if data is not None:
            return data, source
    return None, None


def extract_gps_points(ulog_path: str, source_name: Optional[str] = None) -> List[Tuple[float, float, Optional[float]]]:
    """Load GPS positions from a ULog file."""
    ulog = ULog(ulog_path)
    data, source = get_gps_source(ulog, source_name)

    if data is None or source is None:
        available = sorted({d.name for d in ulog.data_list})
        raise ValueError(
            'No supported GPS source found in ULog. Available messages: ' + ', '.join(available)
        )

    _, lat_field, lon_field, alt_field = source
    if lat_field not in data.data or lon_field not in data.data:
        raise ValueError(f'ULog source "{source[0]}" does not contain {lat_field}/{lon_field}')

    latitudes = data.data[lat_field]
    longitudes = data.data[lon_field]
    altitudes = data.data.get(alt_field)

    points: List[Tuple[float, float, Optional[float]]] = []
    for i in range(len(latitudes)):
        lat = float(latitudes[i])
        lon = float(longitudes[i])
        if math.isnan(lat) or math.isnan(lon):
            continue
        alt = None
        if altitudes is not None:
            try:
                alt_value = float(altitudes[i])
                alt = alt_value if not math.isnan(alt_value) else None
            except (TypeError, ValueError):
                alt = None
        points.append((lat, lon, alt))

    if not points:
        raise ValueError(f'No valid GPS positions found in ULog source "{source[0]}"')

    print(f'Using ULog source: {source[0]} ({len(points)} raw points)')
    return points


def filter_points(points: Sequence[Tuple[float, float, Optional[float]]], step_m: float) -> List[Tuple[float, float, Optional[float]]]:
    """Ignore points that are closer than the configured step in meters."""
    if step_m <= 0.0:
        return list(points)

    filtered: List[Tuple[float, float, Optional[float]]] = []
    last_point: Optional[Tuple[float, float, Optional[float]]] = None

    for point in points:
        if last_point is None:
            filtered.append(point)
            last_point = point
            continue

        distance = haversine_distance(last_point[0], last_point[1], point[0], point[1])
        if distance >= step_m:
            filtered.append(point)
            last_point = point

    return filtered


def save_ulg_mission(points: Sequence[Tuple[float, float, Optional[float]]], output_path: str, altitude: float, cruise_speed: float) -> None:
    """Write the filtered GPS points into a QGroundControl mission plan."""
    if len(points) < 2:
        raise ValueError('At least two GPS points are required to generate a mission.')

    # Extract home position from first point and mission waypoints from remaining points
    home_lat, home_lon, _ = points[0]
    planned_home = (home_lat, home_lon)
    
    # Convert remaining points to lon, lat format for the mission
    lon_lat_points = [(lon, lat) for lat, lon, _ in points[1:]]
    
    saved = save_path_to_qgc_plan(lon_lat_points, output_path, altitude_m=altitude, cruise_speed=cruise_speed, planned_home_position=planned_home)
    if not saved:
        raise RuntimeError('Failed to save QGroundControl mission plan.')

    print(f'Mission saved: {output_path}')
    print(f'Home position: lat={home_lat:.6f}, lon={home_lon:.6f}')
    print(f'Waypoints written: {len(lon_lat_points)}')


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Convert PX4 .ulg GPS positions into a QGroundControl .plan mission file.'
    )
    parser.add_argument('input_ulg', help='Path to input PX4 .ulg file')
    parser.add_argument('-o', '--output', help='Output .plan mission file (default: <input>_mission.plan)')
    parser.add_argument('--step', type=float, default=1.0,
                        help='Minimum distance in meters between consecutive waypoints (default: 1.0)')
    parser.add_argument('-a', '--altitude', type=float, default=20.0,
                        help='Mission waypoint altitude in meters (default: 20.0)')
    parser.add_argument('-s', '--cruise-speed', type=float, default=1.3,
                        help='Mission cruise speed in m/s (default: 1.3)')
    parser.add_argument('--source', choices=[s[0] for s in GPS_SOURCES],
                        help='Preferred GPS data source inside the ULog file')
    args = parser.parse_args()

    if not os.path.exists(args.input_ulg):
        print(f'Error: Input file not found: {args.input_ulg}')
        return 1

    output_path = args.output if args.output else os.path.splitext(args.input_ulg)[0] + '_mission.plan'

    try:
        points = extract_gps_points(args.input_ulg, args.source)
        filtered = filter_points(points, args.step)
        print(f'Filtered waypoints: {len(filtered)} (step >= {args.step} m)')
        if len(filtered) < 2:
            print('Error: Filtered waypoint set contains fewer than 2 points.')
            return 1
        save_ulg_mission(filtered, output_path, args.altitude, args.cruise_speed)
    except Exception as exc:
        print('Error:', exc)
        return 1

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
