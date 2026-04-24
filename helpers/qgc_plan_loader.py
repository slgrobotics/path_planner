#!/usr/bin/env python3

from __future__ import annotations

import json
from pathlib import Path
from typing import List

from helpers.common_data import (
    BreachReturnPoint,
    GeofenceCollection,
    GeofenceZoneCircle,
    GeofenceZonePolygon,
)


def load_geofence_collection_from_qgc_plan(file_path: str) -> GeofenceCollection:
    path = Path(file_path)
    if not path.is_file():
        raise FileNotFoundError(f"QGC plan file not found: '{file_path}'")

    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    if not isinstance(data, dict):
        raise ValueError("QGC plan file must contain a top-level JSON object.")

    if data.get("fileType") != "Plan":
        raise ValueError("QGC plan file must have fileType='Plan'.")

    geo_fence = data.get("geoFence")
    if not isinstance(geo_fence, dict):
        raise ValueError("QGC plan file does not contain a valid 'geoFence' object.")

    polygons: List[GeofenceZonePolygon] = []
    for i, item in enumerate(geo_fence.get("polygons", [])):
        if not isinstance(item, dict):
            continue

        raw_polygon = item.get("polygon")
        inclusion = bool(item.get("inclusion", False))

        if not isinstance(raw_polygon, list):
            continue

        points = []
        for j, pt in enumerate(raw_polygon):
            if not isinstance(pt, list) or len(pt) < 2:
                raise ValueError(f"Polygon {i} point {j} must be [lat, lon].")
            points.append((float(pt[0]), float(pt[1])))

        if len(points) < 3:
            raise ValueError(f"Polygon {i} must contain at least 3 points.")

        polygons.append(
            GeofenceZonePolygon(
                zone_name=f"polygon_{i}",
                points=points,
                inclusion=inclusion,
                reference_frame="wgs84",
            )
        )

    circles: List[GeofenceZoneCircle] = []
    for i, item in enumerate(geo_fence.get("circles", [])):
        if not isinstance(item, dict):
            continue

        raw_circle = item.get("circle")
        inclusion = bool(item.get("inclusion", False))

        if not isinstance(raw_circle, dict):
            continue

        center = raw_circle.get("center")
        radius = raw_circle.get("radius")

        if not isinstance(center, list) or len(center) < 2:
            raise ValueError(f"Circle {i} center must be [lat, lon].")

        circles.append(
            GeofenceZoneCircle(
                zone_name=f"circle_{i}",
                center=(float(center[0]), float(center[1])),
                radius_m=float(radius),
                inclusion=inclusion,
                reference_frame="wgs84",
            )
        )

    breach_return = None
    raw_breach = geo_fence.get("breachReturn")
    if isinstance(raw_breach, list) and len(raw_breach) >= 3:
        breach_return = BreachReturnPoint(
            point=(float(raw_breach[0]), float(raw_breach[1])),
            altitude_m=float(raw_breach[2]),
            reference_frame="wgs84",
        )

    return GeofenceCollection(
        source_name=path.stem,
        reference_frame="wgs84",
        polygons=polygons,
        circles=circles,
        breach_return=breach_return,
    )
