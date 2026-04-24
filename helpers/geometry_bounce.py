#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple
import math

from helpers.geometry_utils import point_in_polygon

# ---------------------------------------------------------------------------------------------
#
# See https://chatgpt.com/s/t_69dc4159a75081919fbffd222699726c
#     https://chatgpt.com/s/t_69dc424ab0288191894a96b510d8c53b
#
# -- Test it:
# cd ~/robot_ws/src/ros2_geofence_manager/test
# ./test_bounce.py --file ../plans/geofence_polygon.yaml --x 1.2 --y 3.2 --angle-deg 30 --angle-jitter 10
#
# `geometry_bounce.py` is a pure-geometry helper module for generating an interior “bounce” target when a robot approaches a geofence boundary.
#
# Its purpose is to support simple wandering, patrolling, and recovery behaviors inside a polygonal operating area without embedding navigation logic into the geofence node itself.
#
# The module works by:
#
# * finding the nearest point on the polygon boundary to the robot
# * identifying the nearest boundary segment
# * computing that segment’s tangent and the polygon’s inward normal
# * constructing a travel direction from a configurable bounce angle
# * ray-casting across the polygon interior
# * returning a safe target point backed off from the far-side boundary
#
# A bounce angle of:
#
# * `0°` means travel straight inward, orthogonal to the nearest boundary
# * positive angles slant along the boundary tangent in one direction
# * negative angles slant in the other direction
#
# The module is intentionally ROS-independent. It operates only on 2D points and polygons, making it easy to unit test and reuse from:
#
# * geofence services
# * patrol logic
# * wandering behaviors
# * BT helper nodes
#
# It exposes two main concepts:
#
# * `BoundaryHit`
#   Describes the nearest boundary contact geometry:
#
#   * closest boundary point
#   * nearest segment index
#   * tangent unit vector
#   * inward normal unit vector
#   * distance to boundary
#
# * `BounceTargetResult`
#   Describes the computed traversal target:
#
#   * success flag
#   * target point inside the polygon
#   * nearest boundary point
#   * far-side boundary intersection point
#   * chosen travel direction
#   * originating segment index
#   * reason string
#
# Primary use cases:
#
# * when the robot is near the geofence and should head back across the interior
# * when a patrol behavior wants a simple “go to the other side” target
# * when an outside/edge recovery behavior needs a directional target inside the fence
#
# Design assumptions:
#
# * best suited for convex polygons
# * especially effective for rectangular or simple yard-like geofences
# * uses segment-based geometry rather than nearest-vertex logic for stability near corners
# * keeps an inset from both the starting boundary and the far boundary to avoid fence-hugging
#
# In short, this module provides the geometric foundation for “bounce off the boundary and continue moving safely inside the allowed area.”
# ---------------------------------------------------------------------------------------------

from helpers.common_data import Point2D, BoundaryHit, BounceTargetResult, EPS


def compute_nearest_boundary_hit(robot_xy: Point2D, polygon: Sequence[Point2D]) -> BoundaryHit:
    """
    Find the nearest point on the polygon boundary and return local boundary geometry.

    Assumptions:
    - polygon has at least 3 vertices
    - polygon vertices are ordered consistently (CW or CCW)
    """
    if len(polygon) < 3:
        raise ValueError("Polygon must contain at least 3 vertices.")

    signed_area = polygon_signed_area(polygon)
    if abs(signed_area) < EPS:
        raise ValueError("Polygon area is too small or degenerate.")

    is_ccw = signed_area > 0.0

    rx, ry = robot_xy

    best_dist_sq = float("inf")
    best_point: Optional[Point2D] = None
    best_segment_index = -1
    best_tangent = (0.0, 0.0)
    best_inward_normal = (0.0, 0.0)

    n = len(polygon)
    for i in range(n):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % n]

        cx, cy, t = closest_point_on_segment_with_t(rx, ry, ax, ay, bx, by)
        dx = cx - rx
        dy = cy - ry
        dist_sq = dx * dx + dy * dy

        if dist_sq < best_dist_sq:
            tx = bx - ax
            ty = by - ay
            tangent = normalize((tx, ty))

            # For CCW polygon, inward normal is left-hand normal.
            # For CW polygon, inward normal is right-hand normal.
            if is_ccw:
                inward = normalize(left_normal(tangent))
            else:
                inward = normalize(right_normal(tangent))

            best_dist_sq = dist_sq
            best_point = (cx, cy)
            best_segment_index = i
            best_tangent = tangent
            best_inward_normal = inward

    assert best_point is not None

    return BoundaryHit(
        closest_point=best_point,
        segment_index=best_segment_index,
        tangent_unit=best_tangent,
        inward_normal_unit=best_inward_normal,
        distance_m=math.sqrt(best_dist_sq),
    )


def compute_bounce_target(
    robot_xy: Point2D,
    polygon: Sequence[Point2D],
    bounce_angle_deg: float = 0.0,
    start_inset_m: float = 0.25,
    goal_inset_m: float = 0.50,
    center_bias: float = 0.1,
) -> BounceTargetResult:
    """
    Compute an interior bounce target.

    Behavior:
    - If robot_xy is inside the polygon:
        use bounce_angle_deg normally
        and target the far interior point (effective center_bias = 1.0)
    - If robot_xy is outside the polygon:
        ignore bounce_angle_deg for this step
        use orthogonal inward recovery
        and target an interior-biased point using center_bias

    center_bias:
      Used only for outside-start recovery.
      0.0 -> stay near the inward start side
      0.5 -> target the middle of the interior ray segment
      1.0 -> target near the far boundary
    """
    if len(polygon) < 3:
        return BounceTargetResult(
            success=False,
            target_point=robot_xy,
            boundary_point=robot_xy,
            far_boundary_point=robot_xy,
            travel_direction_unit=(0.0, 0.0),
            segment_index=-1,
            used_recovery_mode=False,
            reason="polygon must contain at least 3 vertices",
        )

    if start_inset_m < 0.0 or goal_inset_m < 0.0:
        return BounceTargetResult(
            success=False,
            target_point=robot_xy,
            boundary_point=robot_xy,
            far_boundary_point=robot_xy,
            travel_direction_unit=(0.0, 0.0),
            segment_index=-1,
            used_recovery_mode=False,
            reason="inset distances must be non-negative",
        )

    center_bias = max(0.0, min(1.0, center_bias))

    hit = compute_nearest_boundary_hit(robot_xy, polygon)

    inside = point_in_polygon(robot_xy[0], robot_xy[1], polygon)

    # Outside starts use orthogonal inward recovery first.
    effective_bounce_angle_deg = bounce_angle_deg if inside else 0.0
    effective_center_bias = 1.0 if inside else center_bias

    theta = math.radians(effective_bounce_angle_deg)

    nx, ny = hit.inward_normal_unit
    tx, ty = hit.tangent_unit

    dir_x = math.cos(theta) * nx + math.sin(theta) * tx
    dir_y = math.cos(theta) * ny + math.sin(theta) * ty

    try:
        direction = normalize((dir_x, dir_y))
    except ValueError:
        return BounceTargetResult(
            success=False,
            target_point=robot_xy,
            boundary_point=hit.closest_point,
            far_boundary_point=hit.closest_point,
            travel_direction_unit=(0.0, 0.0),
            segment_index=hit.segment_index,
            used_recovery_mode=not inside,
            reason="failed to construct travel direction",
        )

    # Always start slightly inside from the nearest boundary point.
    start_point = (
        hit.closest_point[0] + start_inset_m * nx,
        hit.closest_point[1] + start_inset_m * ny,
    )

    intersections = ray_polygon_intersections(start_point, direction, polygon)

    forward_hits = [
        (s, pt, seg_idx)
        for s, pt, seg_idx in intersections
        if s > max(goal_inset_m, 1e-6)
    ]

    if not forward_hits:
        return BounceTargetResult(
            success=False,
            target_point=start_point,
            boundary_point=hit.closest_point,
            far_boundary_point=start_point,
            travel_direction_unit=direction,
            segment_index=hit.segment_index,
            used_recovery_mode=not inside,
            reason="ray does not intersect polygon boundary ahead",
        )

    _, far_pt, _ = max(forward_hits, key=lambda item: item[0])

    # Safe interior point near the far side.
    far_interior_point = (
        far_pt[0] - goal_inset_m * direction[0],
        far_pt[1] - goal_inset_m * direction[1],
    )

    # Outside recovery may target somewhere inside the chord.
    # Inside bounce targets the full far interior point.
    target_point = (
        start_point[0] + effective_center_bias * (far_interior_point[0] - start_point[0]),
        start_point[1] + effective_center_bias * (far_interior_point[1] - start_point[1]),
    )

    return BounceTargetResult(
        success=True,
        target_point=target_point,
        boundary_point=hit.closest_point,
        far_boundary_point=far_pt,
        travel_direction_unit=direction,
        segment_index=hit.segment_index,
        used_recovery_mode=not inside,
        reason="ok" if inside else "ok (outside start recovered inward)",
    )


def polygon_signed_area(polygon: Sequence[Point2D]) -> float:
    area2 = 0.0
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        area2 += x1 * y2 - x2 * y1
    return 0.5 * area2


def normalize(v: Point2D) -> Point2D:
    x, y = v
    norm = math.hypot(x, y)
    if norm < EPS:
        raise ValueError("Cannot normalize zero-length vector.")
    return (x / norm, y / norm)


def left_normal(v: Point2D) -> Point2D:
    x, y = v
    return (-y, x)


def right_normal(v: Point2D) -> Point2D:
    x, y = v
    return (y, -x)


def closest_point_on_segment_with_t(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> Tuple[float, float, float]:
    abx = bx - ax
    aby = by - ay
    ab_len_sq = abx * abx + aby * aby

    if ab_len_sq < EPS:
        return ax, ay, 0.0

    t = ((px - ax) * abx + (py - ay) * aby) / ab_len_sq
    t = max(0.0, min(1.0, t))

    cx = ax + t * abx
    cy = ay + t * aby
    return cx, cy, t


def ray_polygon_intersections(
    ray_origin: Point2D,
    ray_dir_unit: Point2D,
    polygon: Sequence[Point2D],
) -> List[Tuple[float, Point2D, int]]:
    """
    Return forward intersections between a ray and polygon edges.

    Returns tuples:
      (ray_parameter_s, intersection_point, segment_index)

    where:
      intersection = ray_origin + s * ray_dir_unit
      s >= 0 for forward intersections
    """
    ox, oy = ray_origin
    dx, dy = ray_dir_unit

    hits: List[Tuple[float, Point2D, int]] = []
    n = len(polygon)

    for i in range(n):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % n]

        ex = bx - ax
        ey = by - ay

        # Solve:
        # ray_origin + s * d = A + u * e
        # for s >= 0 and 0 <= u <= 1
        det = cross2d((dx, dy), (ex, ey))
        if abs(det) < EPS:
            continue  # parallel

        qx = ax - ox
        qy = ay - oy

        s = cross2d((qx, qy), (ex, ey)) / det
        u = cross2d((qx, qy), (dx, dy)) / det

        if s >= 0.0 and -EPS <= u <= 1.0 + EPS:
            ix = ox + s * dx
            iy = oy + s * dy
            hits.append((s, (ix, iy), i))

    hits = deduplicate_hits(hits)
    return hits


def deduplicate_hits(hits: List[Tuple[float, Point2D, int]], tol: float = 1e-6) -> List[Tuple[float, Point2D, int]]:
    deduped: List[Tuple[float, Point2D, int]] = []
    for hit in sorted(hits, key=lambda h: h[0]):
        _, pt, _ = hit
        if not deduped:
            deduped.append(hit)
            continue

        _, prev_pt, _ = deduped[-1]
        if math.hypot(pt[0] - prev_pt[0], pt[1] - prev_pt[1]) > tol:
            deduped.append(hit)

    return deduped


def cross2d(a: Point2D, b: Point2D) -> float:
    return a[0] * b[1] - a[1] * b[0]
