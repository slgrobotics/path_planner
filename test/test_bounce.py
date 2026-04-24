#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import random
import signal
import sys
from pathlib import Path
from typing import List, Sequence, Tuple

import cv2
import numpy as np

sys.path.append(str(Path(__file__).resolve().parents[1]))

from helpers.common_data import Point2D
from helpers.geometry_utils import point_in_polygon
from helpers.geometry_bounce import (
    compute_bounce_target,
    compute_nearest_boundary_hit,
)
from helpers.wgs84_to_local import latlon_to_local_xy
from helpers.geofence_loader import load_geofence_as_local_cartesian


#
# Note: there is a newer test script (test_bounce_excl.py) for exclusion zones avoidance behavior, using randomized target selection.
#       this simplified test focuses on inclusion polygons and uses deterministic target planning ("bouncing").
#
# cd ~/planner_ws/path_planner/test
#     using custom YAML format with local Cartesian input:
# ./test_bounce.py --file ../plans/geofence_polygon.yaml --x 1.2 --y 3.2 --angle-deg 30 --angle-jitter 10
# ./test_bounce.py --file ../plans/geofence_polygon.yaml --x 1.2 --y 3.2 --angle-deg 30 --angle-jitter 10 --zone-name home_area
#
#      using QGroundControl .plan files with WGS84 lat/lon input:
# ./test_bounce.py --file ../plans/geofence_qgroundcontrol.plan --x 33.19983710 --y -86.29979086 --angle-deg 30 --angle-jitter 10
# ./test_bounce.py --file ../plans/geofence_qgroundcontrol_multi.plan --x 33.19983710 --y -86.29979086 --angle-deg 30 --angle-jitter 10 --zone-name polygon_1
#


_RUNNING = True


def _handle_sigint(signum, frame):
    global _RUNNING
    _RUNNING = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Standalone geofence bounce visualizer."
    )
    parser.add_argument(
        "--file",
        required=True,
        help="Path to geofence polygon YAML or QGroundControl PLAN file.",
    )
    parser.add_argument(
        "--zone-name",
        default="",
        help="Name of polygon zone to test. Defaults to first inclusion polygon.",
    )
    parser.add_argument(
        "--x",
        type=float,
        required=True,
        help="Initial robot x in polygon frame.",
    )
    parser.add_argument(
        "--y",
        type=float,
        required=True,
        help="Initial robot y in polygon frame.",
    )
    parser.add_argument(
        "--angle-deg",
        type=float,
        default=0.0,
        help="Bounce angle in degrees. 0 = orthogonal inward.",
    )
    parser.add_argument(
        "--angle-jitter",
        type=float,
        default=10.0,
        help="Random variation added to bounce angle (deg)",
    )
    parser.add_argument(
        "--start-inset",
        type=float,
        default=0.25,
        help="Inset from nearest boundary before ray cast.",
    )
    parser.add_argument(
        "--goal-inset",
        type=float,
        default=0.50,
        help="Inset from far boundary for final target.",
    )
    parser.add_argument(
        "--step-size",
        type=float,
        default=0.08,
        help="Motion step size per frame in map units.",
    )
    parser.add_argument(
        "--arrival-tol",
        type=float,
        default=0.12,
        help="Distance threshold for considering target reached.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="Visualization rate.",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1000,
        help="Window width in pixels.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=1000,
        help="Window height in pixels.",
    )
    return parser.parse_args()


def select_test_polygon(collection, zone_name: str = ""):
    if zone_name:
        for poly in collection.polygons:
            if poly.zone_name == zone_name:
                return poly
        raise ValueError(f"Polygon zone '{zone_name}' not found.")

    for poly in collection.polygons:
        if poly.inclusion:
            return poly

    if collection.polygons:
        return collection.polygons[0]

    raise ValueError("No polygon zones found in geofence collection.")


def compute_bounds(points: Sequence[Point2D], margin_ratio: float = 0.1) -> Tuple[float, float, float, float]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)

    span_x = max(max_x - min_x, 1.0)
    span_y = max(max_y - min_y, 1.0)

    margin_x = span_x * margin_ratio
    margin_y = span_y * margin_ratio

    return (
        min_x - margin_x,
        max_x + margin_x,
        min_y - margin_y,
        max_y + margin_y,
    )


def world_to_image(
    p: Point2D,
    bounds: Tuple[float, float, float, float],
    width: int,
    height: int,
) -> Tuple[int, int]:
    min_x, max_x, min_y, max_y = bounds

    sx = (p[0] - min_x) / (max_x - min_x)
    sy = (p[1] - min_y) / (max_y - min_y)

    px = int(round(sx * (width - 1)))
    py = int(round((1.0 - sy) * (height - 1)))
    return px, py


def draw_polygon(
    image: np.ndarray,
    polygon: Sequence[Point2D],
    bounds: Tuple[float, float, float, float],
) -> None:
    pts = [world_to_image(p, bounds, image.shape[1], image.shape[0]) for p in polygon]
    pts_np = np.array(pts, dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(image, [pts_np], isClosed=True, color=(200, 200, 200), thickness=2)


def draw_point(
    image: np.ndarray,
    p: Point2D,
    bounds: Tuple[float, float, float, float],
    color: Tuple[int, int, int],
    radius: int = 5,
    filled: bool = True,
) -> None:
    center = world_to_image(p, bounds, image.shape[1], image.shape[0])
    thickness = -1 if filled else 1
    cv2.circle(image, center, radius, color, thickness)


def draw_line(
    image: np.ndarray,
    a: Point2D,
    b: Point2D,
    bounds: Tuple[float, float, float, float],
    color: Tuple[int, int, int],
    thickness: int = 1,
) -> None:
    pa = world_to_image(a, bounds, image.shape[1], image.shape[0])
    pb = world_to_image(b, bounds, image.shape[1], image.shape[0])
    cv2.line(image, pa, pb, color, thickness)


def draw_text(image: np.ndarray, text: str, x: int, y: int) -> None:
    cv2.putText(
        image,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (220, 220, 220),
        1,
        cv2.LINE_AA,
    )


def normalize(v: Point2D) -> Point2D:
    n = math.hypot(v[0], v[1])
    if n <= 1e-12:
        return (0.0, 0.0)
    return (v[0] / n, v[1] / n)


def dist(a: Point2D, b: Point2D) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, value))


def main() -> int:
    global _RUNNING

    args = parse_args()
    signal.signal(signal.SIGINT, _handle_sigint)

    geofence_file_path = Path(args.file)
    geofence, local_frame = load_geofence_as_local_cartesian(str(geofence_file_path), frame_id="map")

    print("\n=== Geofence Collection Loaded ===")
    print(f"file:            {geofence_file_path}")
    print(f"source_name:     {geofence.source_name}")
    print(f"reference_frame: {geofence.reference_frame}")
    if local_frame is not None:
        print(
            f"local origin (lat, lon): "
            f"({local_frame.origin_lat_deg:.8f}, {local_frame.origin_lon_deg:.8f}) "
            f"frame_id: {local_frame.frame_id}"
        )

    if geofence.polygons:
        print(f"\n--- Polygons: {len(geofence.polygons)} ---")
        for poly in geofence.polygons:
            print(
                f"  {poly.zone_name:20s} "
                f"inclusion={str(poly.inclusion):5s} "
                f"points={len(poly.points)}"
            )
    else:
        print("\n--- Polygons:  (none)")


    if geofence.circles:
        print(f"\n--- Circles: {len(geofence.circles)} ---")
        for circle in geofence.circles:
            print(
                f"  {circle.zone_name:20s} "
                f"inclusion={str(circle.inclusion):5s} "
                f"radius={circle.radius_m:.3f}"
            )
    else:
        print("\n--- Circles: (none)")

    # Optional: breach return
    if geofence.breach_return is not None:
        print("\n--- Breach Return ---")
        print(
            f"  point={geofence.breach_return.point} "
            f"altitude_m={geofence.breach_return.altitude_m:.2f}"
        )
    else:
        print("\n--- Breach Return: (none)")


    selected_polygon = select_test_polygon(geofence, args.zone_name)
    polygon_type = "Inclusion" if selected_polygon.inclusion else "Exclusion"

    print("\n=== Selected Polygon ===")
    print(f"zone_name:       {selected_polygon.zone_name}")
    print(f"inclusion:       {selected_polygon.inclusion}")
    print(f"reference_frame: {selected_polygon.reference_frame}")
    print(f"num points:      {len(selected_polygon.points)}")
    for i, p in enumerate(selected_polygon.points[:5]):
        print(f"  pt[{i}]: {p}")

    if len(selected_polygon.points) > 5:
        print("  ...")

    # Compute bounds for visibility check
    xs = [p[0] for p in selected_polygon.points]
    ys = [p[1] for p in selected_polygon.points]

    print(f"x range: [{min(xs):.6f}, {max(xs):.6f}]")
    print(f"y range: [{min(ys):.6f}, {max(ys):.6f}]")
    print("========================\n")

    if local_frame is not None:
        # Input x/y for .plan files is still given as lat/lon in the current convention.
        robot_xy = latlon_to_local_xy(
            lat_deg=args.x,
            lon_deg=args.y,
            origin_lat_deg=local_frame.origin_lat_deg,
            origin_lon_deg=local_frame.origin_lon_deg,
        )
        print(
            f"Initial robot WGS84: ({args.x:.8f}, {args.y:.8f}) -> "
            f"local XY: ({robot_xy[0]:.3f}, {robot_xy[1]:.3f})"
        )
    else:
        robot_xy = (args.x, args.y)

    polygon: List[Point2D] = list(selected_polygon.points)

    if len(polygon) < 3:
        print("Polygon must contain at least 3 points.", file=sys.stderr)
        return 1

    bounds = compute_bounds(polygon)

    print(f"Initial robot position: ({robot_xy[0]:.6f}, {robot_xy[1]:.6f})")

    inside = point_in_polygon(robot_xy[0], robot_xy[1], selected_polygon.points)
    print(f"Initial inside polygon: {inside}")

    if not selected_polygon.inclusion and not inside:
        print(
            "\nWARNING: Selected zone is an exclusion polygon and the robot starts outside it.\n"
            "         Bounce target is not meaningful in this case.\n"
        )

    trail: List[Point2D] = [robot_xy]

    bounce_angle_deg = args.angle_deg
    bounce_angle_sign = 1.0

    bounce = compute_bounce_target(
        robot_xy=robot_xy,
        polygon=polygon,
        bounce_angle_deg=bounce_angle_deg,
        start_inset_m=args.start_inset,
        goal_inset_m=args.goal_inset,
        center_bias=0.1,
    )
    if not bounce.success:
        print(f"Initial bounce target failed: {bounce.reason}", file=sys.stderr)
        return 1

    target_xy: Point2D = bounce.target_point

    cv2.namedWindow("Geofence Bounce Test", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Geofence Bounce Test", args.width, args.height)

    wait_ms = max(1, int(round(1000.0 / args.fps)))

    while _RUNNING:
        d = dist(robot_xy, target_xy)
        if d <= args.arrival_tol:
            bounce = compute_bounce_target(
                robot_xy=robot_xy,
                polygon=polygon,
                bounce_angle_deg=bounce_angle_deg,
                start_inset_m=args.start_inset,
                goal_inset_m=args.goal_inset,
                center_bias=0.1,
            )
            if not bounce.success:
                print(f"Bounce target failed: {bounce.reason}", file=sys.stderr)
                break
            target_xy = bounce.target_point

            # Invert bounce angle for next run to bounce to both sides, add random jitter.
            #bounce_angle_sign = -bounce_angle_sign
            bounce_angle_sign = 1.0 if random.random() < 0.5 else -1.0
            #bounce_angle_sign = -1.0

            bounce_angle_deg = args.angle_deg * bounce_angle_sign + random.uniform(-args.angle_jitter, args.angle_jitter)
            bounce_angle_deg = clamp(bounce_angle_deg, -60.0, 60.0)

            # bounce_angle_deg += random.uniform(-args.angle_jitter, args.angle_jitter)
            # bounce_angle_deg = clamp(bounce_angle_deg, -60.0, 60.0)
            # bounce_angle_deg = abs(bounce_angle_deg) * bounce_angle_sign

        direction = normalize((target_xy[0] - robot_xy[0], target_xy[1] - robot_xy[1]))
        robot_xy = (
            robot_xy[0] + direction[0] * args.step_size,
            robot_xy[1] + direction[1] * args.step_size,
        )
        trail.append(robot_xy)
        if len(trail) > 3000:
            trail = trail[-3000:]

        hit = compute_nearest_boundary_hit(robot_xy, polygon)
        bounce_dbg = compute_bounce_target(
            robot_xy=robot_xy,
            polygon=polygon,
            bounce_angle_deg=bounce_angle_deg,
            start_inset_m=args.start_inset,
            goal_inset_m=args.goal_inset,
            center_bias=0.1,
        )

        frame = np.zeros((args.height, args.width, 3), dtype=np.uint8)

        draw_polygon(frame, polygon, bounds)

        if len(trail) >= 2:
            for i in range(1, len(trail)):
                draw_line(frame, trail[i - 1], trail[i], bounds, (90, 90, 255), 1)

        draw_point(frame, robot_xy, bounds, (0, 255, 0), radius=12)
        draw_point(frame, target_xy, bounds, (0, 180, 255), radius=6)

        center = world_to_image(target_xy, bounds, frame.shape[1], frame.shape[0])

        tx, ty = center[0] + 8, center[1] - 8
        draw_text(frame, "Target", tx, ty)

        draw_point(frame, hit.closest_point, bounds, (255, 255, 0), radius=5)
        draw_line(frame, robot_xy, hit.closest_point, bounds, (255, 255, 0), 1)

        if bounce_dbg.success:
            draw_point(frame, bounce_dbg.far_boundary_point, bounds, (255, 0, 255), radius=5)
            draw_line(frame, bounce_dbg.boundary_point, bounce_dbg.far_boundary_point, bounds, (120, 120, 120), 1)
            draw_line(frame, robot_xy, bounce_dbg.target_point, bounds, (0, 180, 255), 1)

        draw_text(frame, f"zone name: {selected_polygon.zone_name} type: {polygon_type}", 15, 25)
        draw_text(frame, f"ref frame: {selected_polygon.reference_frame}", 15, 50)
        draw_text(frame, "q: quit", frame.shape[1] - 125, frame.shape[0] - 25)

        draw_text(frame, f"robot: ({robot_xy[0]:.2f}, {robot_xy[1]:.2f})", 15, frame.shape[0] - 75)
        draw_text(frame, f"target: ({target_xy[0]:.2f}, {target_xy[1]:.2f})", 15, frame.shape[0] - 50)
        draw_text(frame, f"nearest boundary dist: {hit.distance_m:.2f}", 15, frame.shape[0] - 25)
        draw_text(frame, f"bounce angle: {bounce_angle_deg:.1f} deg", 300, frame.shape[0] - 25)

        if local_frame is not None:
            local_origin = (0.0, 0.0)
            local_origin_px = world_to_image(local_origin, bounds, frame.shape[1], frame.shape[0])
            draw_text(frame, "+ (0,0) WGS84", local_origin_px[0], local_origin_px[1])

        cv2.imshow("Geofence Bounce Test", frame)
        key = cv2.waitKey(wait_ms) & 0xFF
        if key == ord("q"):
            break

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
