#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
from typing import List

import yaml

from helpers.common_data import (
    BreachReturnPoint,
    GeofenceCollection,
    GeofenceZoneCircle,
    GeofenceZonePolygon,
    Point2D,
)


def _parse_point2d(item: object, what: str) -> Point2D:
    if not isinstance(item, dict):
        raise ValueError(f"{what} must be a mapping like {{x: ..., y: ...}}.")

    if "x" not in item or "y" not in item:
        raise ValueError(f"{what} must contain both 'x' and 'y'.")

    try:
        x = float(item["x"])
        y = float(item["y"])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{what} has invalid numeric values: {item}") from exc

    return (x, y)


def load_geofence_collection_from_yaml(file_path: str) -> GeofenceCollection:

    # --------------------------------------------------
    # Load a multi-zone geofence collection from YAML.
    #
    # Expected format:
    #
    # geofence:
    #   source_name: home_geofence
    #   reference_frame: local_cartesian
    #
    #   polygons:
    #     - zone_name: home_area
    #       inclusion: true
    #       points:
    #         - {x: -3.5, y: -3.5}
    #         - {x: -3.5, y:  3.5}
    #         - {x:  3.5, y:  3.5}
    #         - {x:  3.5, y: -3.5}
    #
    #   circles:
    #     - zone_name: tree_keepout
    #       inclusion: false
    #       center: {x: 2.0, y: 1.0}
    #       radius_m: 0.75
    #
    #   breach_return:
    #     point: {x: 0.0, y: 0.0}
    #     altitude_m: 0.0
    # --------------------------------------------------

    path = Path(file_path)
    if not path.is_file():
        raise FileNotFoundError(f"Geofence YAML file not found: '{file_path}'")

    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError("Geofence YAML must contain a top-level mapping.")

    geofence = data.get("geofence")
    if not isinstance(geofence, dict):
        raise ValueError("Geofence YAML must contain a top-level 'geofence' mapping.")

    source_name = str(geofence.get("source_name", path.stem))
    reference_frame = str(geofence.get("reference_frame", "local_cartesian"))

    raw_polygons = geofence.get("polygons", [])
    if not isinstance(raw_polygons, list):
        raise ValueError("'geofence.polygons' must be a list.")

    polygons: List[GeofenceZonePolygon] = []
    for i, item in enumerate(raw_polygons):
        if not isinstance(item, dict):
            raise ValueError(f"Polygon {i} must be a mapping.")

        zone_name = str(item.get("zone_name", f"polygon_{i}"))
        inclusion = bool(item.get("inclusion", False))

        raw_points = item.get("points")
        if not isinstance(raw_points, list):
            raise ValueError(f"'geofence.polygons[{i}].points' must be a list.")

        points: List[Point2D] = []
        for j, pt in enumerate(raw_points):
            points.append(_parse_point2d(pt, f"Polygon {i} point {j}"))

        if len(points) < 3:
            raise ValueError(f"Polygon {i} must contain at least 3 points.")

        polygons.append(
            GeofenceZonePolygon(
                zone_name=zone_name,
                points=points,
                inclusion=inclusion,
                reference_frame=reference_frame,
            )
        )

    raw_circles = geofence.get("circles", [])
    if not isinstance(raw_circles, list):
        raise ValueError("'geofence.circles' must be a list.")

    circles: List[GeofenceZoneCircle] = []
    for i, item in enumerate(raw_circles):
        if not isinstance(item, dict):
            raise ValueError(f"Circle {i} must be a mapping.")

        zone_name = str(item.get("zone_name", f"circle_{i}"))
        inclusion = bool(item.get("inclusion", False))

        center = _parse_point2d(item.get("center"), f"Circle {i} center")

        if "radius_m" not in item:
            raise ValueError(f"Circle {i} must contain 'radius_m'.")

        try:
            radius_m = float(item["radius_m"])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Circle {i} has invalid radius_m: {item.get('radius_m')}") from exc

        if radius_m <= 0.0:
            raise ValueError(f"Circle {i} radius_m must be positive.")

        circles.append(
            GeofenceZoneCircle(
                zone_name=zone_name,
                center=center,
                radius_m=radius_m,
                inclusion=inclusion,
                reference_frame=reference_frame,
            )
        )

    raw_breach = geofence.get("breach_return")
    breach_return = None
    if raw_breach is not None:
        if not isinstance(raw_breach, dict):
            raise ValueError("'geofence.breach_return' must be a mapping.")

        point = _parse_point2d(raw_breach.get("point"), "breach_return.point")

        try:
            altitude_m = float(raw_breach.get("altitude_m", 0.0))
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"breach_return.altitude_m has invalid value: {raw_breach.get('altitude_m')}"
            ) from exc

        breach_return = BreachReturnPoint(
            point=point,
            altitude_m=altitude_m,
            reference_frame=reference_frame,
        )

    if not polygons and not circles and breach_return is None:
        raise ValueError("Geofence YAML must contain at least one polygon, circle, or breach_return.")

    return GeofenceCollection(
        source_name=source_name,
        reference_frame=reference_frame,
        polygons=polygons,
        circles=circles,
        breach_return=breach_return,
    )
