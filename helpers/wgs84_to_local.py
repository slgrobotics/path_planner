#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import List

from helpers.common_data import (
    Point2D,
    GeofenceCollection,
    GeofenceZonePolygon,
    GeofenceZoneCircle,
    BreachReturnPoint,
    LocalFrameDefinition,
    EARTH_RADIUS_M,
)


def latlon_to_local_xy(
    lat_deg: float,
    lon_deg: float,
    origin_lat_deg: float,
    origin_lon_deg: float,
) -> Point2D:
    """
    Convert WGS84 latitude/longitude to local Cartesian x/y in meters.

    Uses an equirectangular approximation:
      x = R * dlon * cos(lat0)
      y = R * dlat

    where:
      x points east
      y points north

    Good enough for small geofence areas.
    """
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    lat0 = math.radians(origin_lat_deg)
    lon0 = math.radians(origin_lon_deg)

    dlat = lat - lat0
    dlon = lon - lon0

    x_m = EARTH_RADIUS_M * dlon * math.cos(lat0)
    y_m = EARTH_RADIUS_M * dlat
    return (x_m, y_m)


def convert_wgs84_collection_to_local(
    geofence_wgs84: GeofenceCollection,
    origin_lat_deg: float | None = None,
    origin_lon_deg: float | None = None,
    frame_id: str = "local_cartesian",
) -> tuple[GeofenceCollection, LocalFrameDefinition]:
    """
    Convert a WGS84 geofence collection into a local Cartesian frame.

    Assumptions:
    - Polygon points are stored as (lat, lon)
    - Circle centers are stored as (lat, lon)
    - Breach return point is stored as (lat, lon)

    Returns:
      - converted GeofenceCollection with geometry in meters
      - LocalFrameDefinition describing the chosen origin
    """
    if geofence_wgs84.reference_frame.lower() != "wgs84":
        raise ValueError(
            f"Expected WGS84 geofence collection, got "
            f"reference_frame='{geofence_wgs84.reference_frame}' instead."
        )

    if (
        not geofence_wgs84.polygons
        and not geofence_wgs84.circles
        and geofence_wgs84.breach_return is None
    ):
        raise ValueError("Geofence collection is empty; nothing to convert.")

    if origin_lat_deg is None or origin_lon_deg is None:
        # Derive a stable local origin from all available geographic points.
        lat_vals: List[float] = []
        lon_vals: List[float] = []

        for poly in geofence_wgs84.polygons:
            for lat, lon in poly.points:
                lat_vals.append(lat)
                lon_vals.append(lon)

        for circle in geofence_wgs84.circles:
            lat, lon = circle.center
            lat_vals.append(lat)
            lon_vals.append(lon)

        if geofence_wgs84.breach_return is not None:
            lat, lon = geofence_wgs84.breach_return.point
            lat_vals.append(lat)
            lon_vals.append(lon)

        if not lat_vals or not lon_vals:
            raise ValueError("Could not derive local origin from empty geofence geometry.")

        origin_lat_deg = sum(lat_vals) / len(lat_vals)
        origin_lon_deg = sum(lon_vals) / len(lon_vals)

    local_polygons: List[GeofenceZonePolygon] = []
    for poly in geofence_wgs84.polygons:
        if len(poly.points) < 3:
            raise ValueError(f"Polygon zone '{poly.zone_name}' must contain at least 3 points.")

        local_points: List[Point2D] = []
        for lat_deg, lon_deg in poly.points:
            local_points.append(
                latlon_to_local_xy(
                    lat_deg=lat_deg,
                    lon_deg=lon_deg,
                    origin_lat_deg=origin_lat_deg,
                    origin_lon_deg=origin_lon_deg,
                )
            )

        local_polygons.append(
            GeofenceZonePolygon(
                zone_name=poly.zone_name,
                points=local_points,
                inclusion=poly.inclusion,
                reference_frame=frame_id,
            )
        )

    local_circles: List[GeofenceZoneCircle] = []
    for circle in geofence_wgs84.circles:
        center_local = latlon_to_local_xy(
            lat_deg=circle.center[0],
            lon_deg=circle.center[1],
            origin_lat_deg=origin_lat_deg,
            origin_lon_deg=origin_lon_deg,
        )

        local_circles.append(
            GeofenceZoneCircle(
                zone_name=circle.zone_name,
                center=center_local,
                radius_m=circle.radius_m,
                inclusion=circle.inclusion,
                reference_frame=frame_id,
            )
        )

    local_breach_return = None
    if geofence_wgs84.breach_return is not None:
        breach_local = latlon_to_local_xy(
            lat_deg=geofence_wgs84.breach_return.point[0],
            lon_deg=geofence_wgs84.breach_return.point[1],
            origin_lat_deg=origin_lat_deg,
            origin_lon_deg=origin_lon_deg,
        )

        local_breach_return = BreachReturnPoint(
            point=breach_local,
            altitude_m=geofence_wgs84.breach_return.altitude_m,
            reference_frame=frame_id,
        )

    local_geofence = GeofenceCollection(
        source_name=geofence_wgs84.source_name,
        reference_frame=frame_id,
        polygons=local_polygons,
        circles=local_circles,
        breach_return=local_breach_return,
    )

    local_frame = LocalFrameDefinition(
        origin_lat_deg=origin_lat_deg,
        origin_lon_deg=origin_lon_deg,
        frame_id=frame_id,
    )

    return local_geofence, local_frame
