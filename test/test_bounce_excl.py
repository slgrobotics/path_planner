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
from helpers.geometry_random_target import compute_random_safe_target
from helpers.geometry_bounce import (
    compute_bounce_target,
    compute_nearest_boundary_hit,
)
from helpers.wgs84_to_local import latlon_to_local_xy
from helpers.geofence_loader import load_geofence_as_local_cartesian


#
# cd ~/planner_ws/path_planner/test
#     using custom YAML format with local Cartesian input:
# ./test_bounce_excl.py --file ../plans/geofence_polygon.yaml --x 1.2 --y 3.2
# ./test_bounce_excl.py --file ../plans/geofence_polygon.yaml --x 1.2 --y 3.2 --zone-name home_area
#
#      using QGroundControl .plan files with WGS84 lat/lon input:
# ./test_bounce_excl.py --file ../plans/geofence_qgroundcontrol.plan --x 33.19983710 --y -86.29979086
# ./test_bounce_excl.py --file ../plans/geofence_qgroundcontrol_multi.plan --x 33.19983710 --y -86.29979086 --zone-name polygon_1
# ./test_bounce_excl.py --file ../plans/geofence_qgroundcontrol_multi2.plan
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
        default=None,
        help="Initial robot x (or latitude for WGS84). If omitted, random point inside polygon is used.",
    )
    parser.add_argument(
        "--y",
        type=float,
        default=None,
        help="Initial robot y (or longitude for WGS84). If omitted, random point inside polygon is used.",
    )
    parser.add_argument(
        "--min-leg-m",
        type=float,
        default=1.0,
        help="Minimum leg length from robot to a target, meters.",
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


def sample_valid_start_point(
    inclusion_polygon,
    exclusion_polygons,
    exclusion_circles,
    max_attempts=1000,
):
    bounds = compute_bounds(inclusion_polygon, margin_ratio=0.0)

    for _ in range(max_attempts):
        p = (
            random.uniform(bounds[0], bounds[1]),
            random.uniform(bounds[2], bounds[3]),
        )

        if not point_in_polygon(p[0], p[1], inclusion_polygon):
            continue

        if any(point_in_polygon(p[0], p[1], poly) for poly in exclusion_polygons):
            continue

        if any(math.hypot(p[0] - c.center[0], p[1] - c.center[1]) <= c.radius_m for c in exclusion_circles):
            continue

        return p

    raise RuntimeError("Failed to sample valid start point.")


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
    color: Tuple[int, int, int] = (200, 200, 200),
    thickness: int = 2,
) -> None:
    pts = [world_to_image(p, bounds, image.shape[1], image.shape[0]) for p in polygon]
    pts_np = np.array(pts, dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(image, [pts_np], isClosed=True, color=color, thickness=thickness)


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


def draw_text(image: np.ndarray, text: str, x: int, y: int, scale: float = 0.7) -> None:
    cv2.putText(
        image,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        fontScale=scale,
        color=(220, 220, 220),
        thickness=1,
        lineType=cv2.LINE_AA,
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


def collect_all_draw_points(geofence) -> List[Point2D]:
    points: List[Point2D] = []

    # All polygon vertices
    for poly in geofence.polygons:
        points.extend(poly.points)

    # Circle extents: left/right/top/bottom of each circle
    for circle in geofence.circles:
        cx, cy = circle.center
        r = circle.radius_m
        points.extend([
            (cx - r, cy),
            (cx + r, cy),
            (cx, cy - r),
            (cx, cy + r),
        ])

    # Breach return point
    if geofence.breach_return is not None:
        points.append(geofence.breach_return.point)

    return points


def draw_circle(
    image: np.ndarray,
    center: Point2D,
    radius_m: float,
    bounds: Tuple[float, float, float, float],
    color: Tuple[int, int, int],
    thickness: int = 2,
) -> None:
    cx, cy = world_to_image(center, bounds, image.shape[1], image.shape[0])

    edge_world = (center[0] + radius_m, center[1])
    ex, ey = world_to_image(edge_world, bounds, image.shape[1], image.shape[0])

    radius_px = max(1, int(round(abs(ex - cx))))
    cv2.circle(image, (cx, cy), radius_px, color, thickness)


def draw_cross(
    image: np.ndarray,
    p: Point2D,
    bounds: Tuple[float, float, float, float],
    color: Tuple[int, int, int],
    size: int = 6,
    thickness: int = 2,
) -> None:
    px, py = world_to_image(p, bounds, image.shape[1], image.shape[0])
    cv2.line(image, (px - size, py), (px + size, py), color, thickness)
    cv2.line(image, (px, py - size), (px, py + size), color, thickness)


def zone_color(inclusion: bool) -> Tuple[int, int, int]:
    return (0, 200, 0) if inclusion else (0, 0, 200)


def polygon_signed_area(polygon: Sequence[Point2D]) -> float:
    area2 = 0.0
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        area2 += x1 * y2 - x2 * y1
    return 0.5 * area2


def left_normal(v: Point2D) -> Point2D:
    x, y = v
    return (-y, x)


def right_normal(v: Point2D) -> Point2D:
    x, y = v
    return (y, -x)


def longest_edge_index(polygon: Sequence[Point2D]) -> int:
    best_i = 0
    best_len = -1.0
    n = len(polygon)
    for i in range(n):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % n]
        d = math.hypot(bx - ax, by - ay)
        if d > best_len:
            best_len = d
            best_i = i
    return best_i


def compute_polygon_label_point(
    polygon: Sequence[Point2D],
    inset_m: float = 0.35,
    edge_index: int = 0,
) -> Point2D:
    """
    Return a point slightly inside the polygon, near the midpoint of one edge.
    """
    if len(polygon) < 3:
        raise ValueError("Polygon must contain at least 3 points.")

    n = len(polygon)
    i = edge_index % n

    ax, ay = polygon[i]
    bx, by = polygon[(i + 1) % n]

    mid = ((ax + bx) * 0.5, (ay + by) * 0.5)

    tx = bx - ax
    ty = by - ay
    tangent = normalize((tx, ty))

    is_ccw = polygon_signed_area(polygon) > 0.0
    inward = normalize(left_normal(tangent) if is_ccw else right_normal(tangent))

    return (
        mid[0] + inset_m * inward[0],
        mid[1] + inset_m * 3.0 * inward[1],
    )


def compute_circle_label_point(
    center: Point2D,
    radius_m: float,
    inset_factor: float = 0.1,
) -> Point2D:
    cx, cy = center
    return (cx - radius_m * inset_factor * 3, cy + radius_m * inset_factor)


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
    if not selected_polygon.inclusion:
        raise ValueError("Selected polygon is an exclusion polygon, cannot proceed.")
    else:
        polygon_type = "Inclusion"

    inclusion_candidates = [poly for poly in geofence.polygons if poly.inclusion]
    if not inclusion_candidates:
        raise ValueError("No inclusion polygon available for random target generation.")

    inclusion_polygon = inclusion_candidates[0].points

    exclusion_polygons = [poly.points for poly in geofence.polygons if not poly.inclusion]
    exclusion_circles = [circle for circle in geofence.circles if not circle.inclusion]

    print("\n=== Selected Polygon ===")
    print(f"zone_name:       {selected_polygon.zone_name}")
    print(f"inclusion:       {selected_polygon.inclusion}")
    print(f"reference_frame: {selected_polygon.reference_frame}")
    print(f"num points:      {len(selected_polygon.points)}")
    for i, p in enumerate(selected_polygon.points[:5]):
        print(f"  pt[{i}]: {p}")

    if len(selected_polygon.points) > 5:
        print("  ...")

    all_draw_points = collect_all_draw_points(geofence)
    if not all_draw_points:
        print("No drawable geometry found.", file=sys.stderr)
        return 1

    xs = [p[0] for p in all_draw_points]
    ys = [p[1] for p in all_draw_points]

    print(f"x range: [{min(xs):.6f}, {max(xs):.6f}]")
    print(f"y range: [{min(ys):.6f}, {max(ys):.6f}]")
    print("========================\n")

    # Validate argument combination
    if (args.x is None) ^ (args.y is None):
        raise ValueError("Both --x and --y must be provided together, or both omitted.")

    if args.x is not None and args.y is not None:
        # User provided position
        if local_frame is not None:
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
    else:
        # Auto-generate start inside selected polygon
        robot_xy = sample_valid_start_point(
            inclusion_polygon,
            exclusion_polygons,
            exclusion_circles,
        )
        print(
            f"Initial robot position randomly sampled inside polygon: "
            f"({robot_xy[0]:.3f}, {robot_xy[1]:.3f})"
        )

    polygon: List[Point2D] = list(selected_polygon.points)

    if len(polygon) < 3:
        print("Polygon must contain at least 3 points.", file=sys.stderr)
        return 1

    bounds = compute_bounds(all_draw_points)

    print(f"Initial robot position: ({robot_xy[0]:.6f}, {robot_xy[1]:.6f})")

    inside = point_in_polygon(robot_xy[0], robot_xy[1], selected_polygon.points)
    print(f"Initial inside polygon: {inside}")

    if not selected_polygon.inclusion and not inside:
        print(
            "\nWARNING: Selected zone is an exclusion polygon and the robot starts outside it.\n"
            "         Bounce target is not meaningful in this case.\n"
        )

    trail: List[Point2D] = [robot_xy]

    inclusion_boundary_margin_m=0.0025
    exclusion_boundary_margin_m=0.515

    target_result = compute_random_safe_target(
        robot_xy=robot_xy,
        inclusion_polygon=inclusion_polygon,
        exclusion_polygons=exclusion_polygons,
        exclusion_circles=exclusion_circles,
        target_not_closer_m=args.min_leg_m,
        max_samples=200,
        inclusion_boundary_margin_m=inclusion_boundary_margin_m,
        exclusion_boundary_margin_m=exclusion_boundary_margin_m,
    )

    if not target_result.success:
        print(f"Initial random target failed: {target_result.reason}", file=sys.stderr)
        return 1

    target_xy = target_result.target_point
    print(f"Initial random target found in {target_result.attempts_used} attempts: {target_xy}")

    cv2.namedWindow("Geofence Bounce Test", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Geofence Bounce Test", args.width, args.height)

    wait_ms = max(1, int(round(1000.0 / args.fps)))

    while _RUNNING:
        d = dist(robot_xy, target_xy)

        if d <= args.arrival_tol:
            target_result = compute_random_safe_target(
                robot_xy=robot_xy,
                inclusion_polygon=inclusion_polygon,
                exclusion_polygons=exclusion_polygons,
                exclusion_circles=exclusion_circles,
                target_not_closer_m=args.min_leg_m,
                max_samples=200,
                inclusion_boundary_margin_m=inclusion_boundary_margin_m,
                exclusion_boundary_margin_m=exclusion_boundary_margin_m,
            )
            if not target_result.success:
                print(f"Random target failed: {target_result.reason}", file=sys.stderr)
                break

            target_xy = target_result.target_point

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
            bounce_angle_deg=30.0,
            start_inset_m=0.25,
            goal_inset_m=0.50,
            center_bias=0.1,
        )

        frame = np.zeros((args.height, args.width, 3), dtype=np.uint8)

        # Draw all polygons - inclusion and exclusion
        for poly in geofence.polygons:
            draw_polygon(frame, poly.points, bounds, color=zone_color(poly.inclusion), thickness=2)
            edge_i = longest_edge_index(poly.points)
            label_world = compute_polygon_label_point(poly.points, inset_m=0.35, edge_index=edge_i)
            lx, ly = world_to_image(label_world, bounds, frame.shape[1], frame.shape[0])
            draw_text(frame, poly.zone_name, lx + 4, ly - 4, scale=0.9)

        # Draw circles
        for circle in geofence.circles:
            draw_circle(frame, circle.center, circle.radius_m, bounds, color=zone_color(circle.inclusion), thickness=2)
            label_world = compute_circle_label_point(circle.center, circle.radius_m, inset_factor=0.1)
            lx, ly = world_to_image(label_world, bounds, frame.shape[1], frame.shape[0])
            draw_text(frame, circle.zone_name, lx + 4, ly - 4, scale=0.9)

        # Draw breach return
        if geofence.breach_return is not None:
            draw_cross(frame, geofence.breach_return.point, bounds, color=(255, 255, 255), size=8, thickness=2)
            bx, by = world_to_image(geofence.breach_return.point, bounds, frame.shape[1], frame.shape[0])
            draw_text(frame, "BreachReturn", bx + 8, by - 8, scale=0.9)

        if len(trail) >= 2:
            for i in range(1, len(trail)):
                draw_line(frame, trail[i - 1], trail[i], bounds, (90, 90, 255), 1)

        draw_point(frame, robot_xy, bounds, (0, 255, 0), radius=12)
        draw_point(frame, target_xy, bounds, (0, 180, 255), radius=6)

        center = world_to_image(target_xy, bounds, frame.shape[1], frame.shape[0])

        tx, ty = center[0] + 8, center[1] - 8
        draw_text(frame, "Target", tx, ty, scale=0.8)

        draw_point(frame, hit.closest_point, bounds, (255, 255, 0), radius=5)
        draw_line(frame, robot_xy, hit.closest_point, bounds, (255, 255, 0), 1)

        if target_result.success:
            # draw_point(frame, bounce_dbg.far_boundary_point, bounds, (255, 0, 255), radius=5)
            # draw_line(frame, bounce_dbg.boundary_point, bounce_dbg.far_boundary_point, bounds, (120, 120, 120), 1)
            # draw_line(frame, robot_xy, bounce_dbg.target_point, bounds, (0, 180, 255), 1)
            pass

        draw_text(frame, f"zone name: {selected_polygon.zone_name} type: {polygon_type}", 15, 25)
        draw_text(frame, f"ref frame: {selected_polygon.reference_frame}", 15, 50)
        draw_text(frame, "q: quit", frame.shape[1] - 80, frame.shape[0] - 25)

        draw_text(frame, f"robot: ({robot_xy[0]:.2f}, {robot_xy[1]:.2f})", 15, frame.shape[0] - 75)
        draw_text(frame, f"target: ({target_xy[0]:.2f}, {target_xy[1]:.2f})", 15, frame.shape[0] - 50)
        draw_text(frame, f"nearest boundary dist: {hit.distance_m:.2f}", 15, frame.shape[0] - 25)

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
