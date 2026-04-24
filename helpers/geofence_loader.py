#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
from typing import Callable

from helpers.common_data import GeofenceCollection, LocalFrameDefinition
from helpers.polygon_loader import load_geofence_collection_from_yaml
from helpers.qgc_plan_loader import load_geofence_collection_from_qgc_plan
from helpers.wgs84_to_local import convert_wgs84_collection_to_local


_LOADER_BY_SUFFIX: dict[str, Callable[[str], GeofenceCollection]] = {
    ".yaml": load_geofence_collection_from_yaml,
    ".yml": load_geofence_collection_from_yaml,
    ".plan": load_geofence_collection_from_qgc_plan,
}


def load_geofence(file_path: str) -> GeofenceCollection:
    """
    Load a geofence collection from a supported file type.

    Supported formats:
    - .yaml / .yml  -> local Cartesian geofence collection
    - .plan         -> QGroundControl geofence collection in WGS84

    Returns:
        GeofenceCollection

    Raises:
        FileNotFoundError:
            If the file does not exist.
        ValueError:
            If the file extension is unsupported or the content is invalid.
    """
    path = Path(file_path)
    if not path.is_file():
        raise FileNotFoundError(f"Geofence file not found: '{file_path}'")

    suffix = path.suffix.lower()
    loader = _LOADER_BY_SUFFIX.get(suffix)
    if loader is None:
        supported = ", ".join(sorted(_LOADER_BY_SUFFIX.keys()))
        raise ValueError(
            f"Unsupported geofence file extension '{suffix}' for file '{file_path}'. "
            f"Supported extensions: {supported}"
        )

    return loader(str(path))


def load_geofence_as_local_cartesian(
    file_path: str,
    frame_id: str = "local_cartesian",
) -> tuple[GeofenceCollection, LocalFrameDefinition | None]:
    """
    Load a geofence collection and, if needed, convert WGS84 geometry into a
    local Cartesian frame.

    Intended for:
    - testing
    - visualization
    - standalone geometry experiments

    Not intended for:
    - live ROS map-aligned runtime use with GPS

    For live ROS use, a QGC .plan should be converted with ROS-aware services
    (e.g. fromLL / toLL) so the resulting geometry is truly aligned with the
    robot's world frame.
    """
    geofence = load_geofence(file_path)

    if geofence.reference_frame.lower() == "wgs84":
        return convert_wgs84_collection_to_local(
            geofence_wgs84=geofence,
            frame_id=frame_id,
        )

    return geofence, None
