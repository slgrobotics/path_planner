from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Literal


Point2D = Tuple[float, float]  # (x, y) in a local Cartesian frame, or (lat, lon) in WGS84 depending on context.

INVALID_DISTANCE_M = -1.0
EPS = 1e-9
EARTH_RADIUS_M = 6378137.0


@dataclass
class LocalFrameDefinition:
    origin_lat_deg: float
    origin_lon_deg: float
    frame_id: str = "map"  # e.g. "map" or "odom", to let ROS know which frame the object is running in. Not the original "reference_frame"


@dataclass
class GeofenceZonePolygon:
    zone_name: str
    points: List[Point2D]
    inclusion: bool
    reference_frame: str = "local_cartesian"  # e.g. "local_cartesian" or "wgs84", to keep track of the frame the object was defined in. Not "frame_id"


@dataclass
class GeofenceZoneCircle:
    zone_name: str
    center: Point2D
    radius_m: float
    inclusion: bool
    reference_frame: str = "local_cartesian"


@dataclass
class BreachReturnPoint:
    point: Point2D
    altitude_m: float
    reference_frame: str = "wgs84"


@dataclass
class GeofenceCollection:
    source_name: str
    reference_frame: str
    polygons: List[GeofenceZonePolygon] = field(default_factory=list)
    circles: List[GeofenceZoneCircle] = field(default_factory=list)
    breach_return: Optional[BreachReturnPoint] = None


@dataclass
class BoundaryContext:
    x: float
    y: float
    inside: bool
    distance_m: float
    closest_point: Point2D
    segment_index: int
    tangent_unit: Point2D
    inward_normal_unit: Point2D


@dataclass
class BoundaryHit:
    closest_point: Point2D
    segment_index: int
    tangent_unit: Point2D
    inward_normal_unit: Point2D
    distance_m: float


@dataclass
class BounceTargetResult:
    success: bool
    target_point: Point2D
    boundary_point: Point2D
    far_boundary_point: Point2D
    travel_direction_unit: Point2D
    segment_index: int
    used_recovery_mode: bool
    reason: str

