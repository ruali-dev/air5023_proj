"""Straight-in station docking using a known LiDAR-visible face."""

from dataclasses import dataclass
import math
from statistics import median

from .tracking import Pose


@dataclass(frozen=True)
class Station:
    pose: Pose
    stand_range: float | None = None
    approach_distance: float = 0.35

    def __post_init__(self):
        distances = (self.approach_distance,) if self.stand_range is None else (
            self.stand_range, self.approach_distance)
        if not all(math.isfinite(value) for value in (self.pose.x, self.pose.y, self.pose.yaw,
                                                       *distances)):
            raise ValueError("Station pose and distances must be finite")
        if (self.stand_range is not None and self.stand_range <= 0) or self.approach_distance <= 0:
            raise ValueError("Station standoff and approach must be positive")

    @property
    def uses_lidar_docking(self):
        return self.stand_range is not None

    @property
    def approach(self):
        dx, dy = math.cos(self.pose.yaw), math.sin(self.pose.yaw)
        return Pose(self.pose.x - self.approach_distance * dx,
                    self.pose.y - self.approach_distance * dy, self.pose.yaw)

    def offsets(self, pose):
        dx, dy = pose.x - self.pose.x, pose.y - self.pose.y
        c, s = math.cos(self.pose.yaw), math.sin(self.pose.yaw)
        return -(c * dx + s * dy), c * dy - s * dx, angle_difference(pose.yaw, self.pose.yaw)


def angle_difference(angle, reference):
    return (angle - reference + math.pi) % (2 * math.pi) - math.pi


def station_depth(station, pose, lidar, sample):
    """Return remaining distance along station heading, in metres.

    Project valid near-forward returns onto the known face normal. Require a
    cluster near the predicted plane so a single ray cannot determine arrival.
    The configured standoff is measured from the robot base origin.
    """
    remaining, lateral, heading = station.offsets(pose)
    if not (-0.18 <= remaining <= station.approach_distance + 0.18
            and abs(lateral) <= 0.12 and abs(heading) <= 0.18):
        return None
    origin = lidar.origin[0] * math.cos(heading) - lidar.origin[1] * math.sin(heading)
    depths = [origin + distance * math.cos(angle + heading) - station.stand_range
              for angle, distance in zip(lidar.angles, sample.ranges)
              if distance is not None and lidar.blind_radius <= distance <= lidar.max_range
              and abs(angle_difference(angle, 0.0)) <= 0.20]
    inliers = [depth for depth in depths if abs(depth - remaining) <= 0.15]
    if len(inliers) < 5:
        return None
    center = median(inliers)
    consistent = [depth for depth in inliers if abs(depth - center) <= 0.035]
    return median(consistent) if len(consistent) >= 5 else None
