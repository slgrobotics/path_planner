#!/usr/bin/env python3

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Sequence, Tuple

from helpers.common_data import EPS, GeofenceZoneCircle, Point2D
from helpers.geometry_bounce import compute_nearest_boundary_hit
from helpers.geometry_utils import point_in_polygon


@dataclass
class RandomTargetResult:
    success: bool
    target_point: Point2D
    reason: str
    attempts_used: int


def compute_random_safe_target(
    robot_xy: Point2D,
    inclusion_polygon: Sequence[Point2D],
    exclusion_polygons: Sequence[Sequence[Point2D]] | None = None,
    exclusion_circles: Sequence[GeofenceZoneCircle] | None = None,
    target_not_closer_m: float = 1.0,
    max_samples: int = 200,
    inclusion_boundary_margin_m: float = 0.25,
    exclusion_boundary_margin_m: float = 0.25,
) -> RandomTargetResult:
    """
    Choose a random target inside the inclusion polygon that:
    - lies inside the inclusion polygon
    - stays away from the inclusion boundary by inclusion_boundary_margin_m
    - does not lie inside any exclusion polygon
    - does not lie inside or too close to any exclusion circle
    - has a direct robot->target segment that does not intersect any exclusion polygon
    - has a direct robot->target segment that does not intersect any exclusion circle

    Intended as a simple high-level target selector for wandering / patrolling.

    Current limitation:
    - the allowed area is defined by one inclusion polygon only
    """
    if len(inclusion_polygon) < 3:
        return RandomTargetResult(
            success=False,
            target_point=robot_xy,
            reason="inclusion polygon must contain at least 3 points",
            attempts_used=0,
        )

    exclusion_polygons = exclusion_polygons or []
    exclusion_circles = exclusion_circles or []

    robot_inside_exclusion_polygon = point_in_any_polygon(robot_xy, exclusion_polygons)
    robot_inside_exclusion_circle = point_in_any_circle(robot_xy, exclusion_circles)

    bbox = polygon_bounding_box(inclusion_polygon)
    diag = math.hypot(bbox[1] - bbox[0], bbox[3] - bbox[2])

    if target_not_closer_m > 0.5 * diag:
        return RandomTargetResult(
                success=False,
                target_point=robot_xy,
                reason=f"target_not_closer_m ({target_not_closer_m}) is too large relative to the inclusion polygon size (diagonal {diag:.2f})",
                attempts_used=0,
            )

    for attempt in range(1, max_samples + 1):
        candidate = sample_random_point_in_bbox(bbox)

        # Reject if too close to robot
        if distance(robot_xy, candidate) < target_not_closer_m:
            continue

        if not point_in_polygon(candidate[0], candidate[1], inclusion_polygon):
            continue

        if candidate_too_close_to_inclusion_boundary(
            candidate,
            inclusion_polygon,
            inclusion_boundary_margin_m,
        ):
            continue

        if candidate_blocked_by_exclusion_polygons(
            candidate,
            exclusion_polygons,
            exclusion_boundary_margin_m,
        ):
            continue

        if candidate_blocked_by_exclusion_circles(
            candidate,
            exclusion_circles,
            exclusion_boundary_margin_m,
        ):
            continue

        if not candidate_has_clear_path(
            robot_xy,
            candidate,
            exclusion_polygons,
            exclusion_circles,
        ):
            continue

        return RandomTargetResult(
            success=True,
            target_point=candidate,
            reason="ok",
            attempts_used=attempt,
        )

    if robot_inside_exclusion_polygon or robot_inside_exclusion_circle:
        fail_reason = (
            "failed to find a valid random target "
            "(robot starts inside an exclusion zone or constraints too strict)"
        )
    else:
        fail_reason = (
            "failed to find a valid random target "
            "(try reducing target_not_closer_m or margins)"
        )

    return RandomTargetResult(
        success=False,
        target_point=robot_xy,
        reason=fail_reason,
        attempts_used=max_samples,
    )


def distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def candidate_too_close_to_inclusion_boundary(
    candidate: Point2D,
    inclusion_polygon: Sequence[Point2D],
    margin_m: float,
) -> bool:
    try:
        outer_hit = compute_nearest_boundary_hit(candidate, inclusion_polygon)
    except ValueError:
        return True

    return outer_hit.distance_m < margin_m


def candidate_blocked_by_exclusion_polygons(
    candidate: Point2D,
    exclusion_polygons: Sequence[Sequence[Point2D]],
    margin_m: float,
) -> bool:
    for exclusion in exclusion_polygons:
        if len(exclusion) < 3:
            continue

        if point_in_polygon(candidate[0], candidate[1], exclusion):
            return True

        try:
            exclusion_hit = compute_nearest_boundary_hit(candidate, exclusion)
        except ValueError:
            continue

        if exclusion_hit.distance_m < margin_m:
            return True

    return False


def candidate_blocked_by_exclusion_circles(
    candidate: Point2D,
    exclusion_circles: Sequence[GeofenceZoneCircle],
    margin_m: float,
) -> bool:
    for circle in exclusion_circles:
        if point_blocked_by_circle(
            candidate,
            circle.center,
            circle.radius_m,
            margin_m,
        ):
            return True
    return False


def candidate_has_clear_path(
    robot_xy: Point2D,
    candidate: Point2D,
    exclusion_polygons: Sequence[Sequence[Point2D]],
    exclusion_circles: Sequence[GeofenceZoneCircle],
) -> bool:
    if path_intersects_any_polygon(robot_xy, candidate, exclusion_polygons):
        return False

    if path_intersects_any_circle(robot_xy, candidate, exclusion_circles):
        return False

    return True


def polygon_bounding_box(
    polygon: Sequence[Point2D],
) -> Tuple[float, float, float, float]:
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    return (min(xs), max(xs), min(ys), max(ys))


def sample_random_point_in_bbox(
    bbox: Tuple[float, float, float, float],
) -> Point2D:
    min_x, max_x, min_y, max_y = bbox
    return (
        random.uniform(min_x, max_x),
        random.uniform(min_y, max_y),
    )


def point_in_any_polygon(
    p: Point2D,
    polygons: Sequence[Sequence[Point2D]],
) -> bool:
    for polygon in polygons:
        if len(polygon) < 3:
            continue
        if point_in_polygon(p[0], p[1], polygon):
            return True
    return False


def path_intersects_any_polygon(
    a: Point2D,
    b: Point2D,
    polygons: Sequence[Sequence[Point2D]],
) -> bool:
    for polygon in polygons:
        if len(polygon) < 3:
            continue

        if point_in_polygon(a[0], a[1], polygon) or point_in_polygon(b[0], b[1], polygon):
            return True

        if segment_intersects_polygon(a, b, polygon):
            return True

    return False


def segment_intersects_polygon(
    a: Point2D,
    b: Point2D,
    polygon: Sequence[Point2D],
) -> bool:
    n = len(polygon)
    for i in range(n):
        c = polygon[i]
        d = polygon[(i + 1) % n]

        if segments_intersect(a, b, c, d):
            return True

    return False


def segments_intersect(
    a: Point2D,
    b: Point2D,
    c: Point2D,
    d: Point2D,
) -> bool:
    o1 = orientation(a, b, c)
    o2 = orientation(a, b, d)
    o3 = orientation(c, d, a)
    o4 = orientation(c, d, b)

    if o1 * o2 < 0.0 and o3 * o4 < 0.0:
        return True

    if abs(o1) < EPS and on_segment(a, c, b):
        return True
    if abs(o2) < EPS and on_segment(a, d, b):
        return True
    if abs(o3) < EPS and on_segment(c, a, d):
        return True
    if abs(o4) < EPS and on_segment(c, b, d):
        return True

    return False


def orientation(
    a: Point2D,
    b: Point2D,
    c: Point2D,
) -> float:
    return cross2d((b[0] - a[0], b[1] - a[1]), (c[0] - a[0], c[1] - a[1]))


def on_segment(
    a: Point2D,
    p: Point2D,
    b: Point2D,
) -> bool:
    return (
        min(a[0], b[0]) - EPS <= p[0] <= max(a[0], b[0]) + EPS
        and min(a[1], b[1]) - EPS <= p[1] <= max(a[1], b[1]) + EPS
    )


def cross2d(
    u: Point2D,
    v: Point2D,
) -> float:
    return u[0] * v[1] - u[1] * v[0]


def point_inside_circle(
    p: Point2D,
    center: Point2D,
    radius_m: float,
) -> bool:
    return math.hypot(p[0] - center[0], p[1] - center[1]) <= radius_m + EPS


def point_blocked_by_circle(
    p: Point2D,
    center: Point2D,
    radius_m: float,
    margin_m: float,
) -> bool:
    return math.hypot(p[0] - center[0], p[1] - center[1]) <= radius_m + margin_m


def point_in_any_circle(
    p: Point2D,
    circles: Sequence[GeofenceZoneCircle],
) -> bool:
    for circle in circles:
        if point_inside_circle(p, circle.center, circle.radius_m):
            return True
    return False


def path_intersects_any_circle(
    a: Point2D,
    b: Point2D,
    circles: Sequence[GeofenceZoneCircle],
) -> bool:
    for circle in circles:
        if point_inside_circle(a, circle.center, circle.radius_m):
            return True
        if point_inside_circle(b, circle.center, circle.radius_m):
            return True
        if segment_intersects_circle(a, b, circle.center, circle.radius_m):
            return True
    return False


def segment_intersects_circle(
    a: Point2D,
    b: Point2D,
    center: Point2D,
    radius_m: float,
) -> bool:
    ax, ay = a
    bx, by = b
    cx, cy = center

    abx = bx - ax
    aby = by - ay
    ab_len_sq = abx * abx + aby * aby

    if ab_len_sq < EPS:
        return math.hypot(ax - cx, ay - cy) <= radius_m + EPS

    t = ((cx - ax) * abx + (cy - ay) * aby) / ab_len_sq
    t = max(0.0, min(1.0, t))

    px = ax + t * abx
    py = ay + t * aby

    return math.hypot(px - cx, py - cy) <= radius_m + EPS
