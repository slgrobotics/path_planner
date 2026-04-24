#!/usr/bin/env python3

from __future__ import annotations

from typing import Iterable, List, Sequence, Tuple
import math


Point2D = Tuple[float, float]


def point_in_polygon(x: float, y: float, polygon: Sequence[Point2D]) -> bool:
    """
    Ray-casting point-in-polygon test.

    Returns True if the point is inside the polygon.
    Points on the boundary are treated as inside.
    """
    if len(polygon) < 3:
        return False

    inside = False
    n = len(polygon)

    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]

        if _point_on_segment(x, y, x1, y1, x2, y2):
            return True

        intersects = ((y1 > y) != (y2 > y))
        if intersects:
            x_intersection = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x_intersection == x:
                return True
            if x_intersection > x:
                inside = not inside

    return inside


def distance_to_polygon_edges(x: float, y: float, polygon: Sequence[Point2D]) -> float:
    """
    Minimum Euclidean distance from a point to polygon boundary.
    """
    if len(polygon) < 2:
        return math.inf

    min_dist = math.inf
    n = len(polygon)

    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        dist = _distance_point_to_segment(x, y, x1, y1, x2, y2)
        min_dist = min(min_dist, dist)

    return min_dist


def closest_point_on_polygon(x: float, y: float, polygon: Sequence[Point2D]) -> Point2D:
    """
    Closest point on the polygon boundary to the given point.
    """
    if len(polygon) < 2:
        return (x, y)

    best_point = (x, y)
    best_dist_sq = math.inf
    n = len(polygon)

    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        px, py = _closest_point_on_segment(x, y, x1, y1, x2, y2)
        dist_sq = (px - x) ** 2 + (py - y) ** 2
        if dist_sq < best_dist_sq:
            best_dist_sq = dist_sq
            best_point = (px, py)

    return best_point


def _closest_point_on_segment(
    px: float,
    py: float,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> Point2D:
    dx = x2 - x1
    dy = y2 - y1

    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq == 0.0:
        return (x1, y1)

    t = ((px - x1) * dx + (py - y1) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))

    return (x1 + t * dx, y1 + t * dy)


def _distance_point_to_segment(
    px: float,
    py: float,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> float:
    cx, cy = _closest_point_on_segment(px, py, x1, y1, x2, y2)
    return math.hypot(px - cx, py - cy)


def _point_on_segment(
    px: float,
    py: float,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    eps: float = 1e-9,
) -> bool:
    cross = (px - x1) * (y2 - y1) - (py - y1) * (x2 - x1)
    if abs(cross) > eps:
        return False

    dot = (px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)
    if dot < -eps:
        return False

    seg_len_sq = (x2 - x1) ** 2 + (y2 - y1) ** 2
    if dot - seg_len_sq > eps:
        return False

    return True