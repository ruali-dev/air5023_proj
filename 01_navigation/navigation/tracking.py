"""Pure pursuit-style waypoint tracking for differential-drive routes."""

from __future__ import annotations

from dataclasses import dataclass
import math


DEFAULT_CRUISE_SPEED = 0.22


@dataclass(frozen=True)
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class Route:
    waypoints: tuple[tuple[float, float], ...]
    goal_yaw: float | None = None

    def __post_init__(self):
        waypoints = tuple(tuple(point) for point in self.waypoints)
        if not waypoints or any(len(point) != 2 or not all(math.isfinite(value) for value in point)
                                for point in waypoints):
            raise ValueError('Route must contain finite two-dimensional waypoints')
        if self.goal_yaw is not None and not math.isfinite(self.goal_yaw):
            raise ValueError('Route goal_yaw must be finite')
        object.__setattr__(self, 'waypoints', waypoints)


@dataclass(frozen=True)
class Command:
    vx: float
    wz: float
    phase: str
    reached: bool


class RouteTracker:
    """Track a sequence of map-frame waypoints with in-place heading alignment."""

    POSITION_TOLERANCE = 0.025
    ALIGNMENT_POSITION_TOLERANCE = 0.04
    WAYPOINT_TOLERANCE = 0.045
    HEADING_TOLERANCE = 0.035
    ALIGN_THRESHOLD = 0.32
    CRUISE_RAMP_START = 0.10
    CRUISE_RAMP_LENGTH = 0.25
    CRUISE_LOOKAHEAD = 0.18
    CROSS_TRACK_GAIN = 2.0
    MAX_CROSS_TRACK_ANGLE = 0.50

    def __init__(self, route: Route, *, max_vx: float, max_wz: float,
                 cruise_speed: float = DEFAULT_CRUISE_SPEED):
        if not math.isfinite(max_vx) or max_vx <= 0:
            raise ValueError("max_vx must be positive and finite")
        if not math.isfinite(max_wz) or max_wz <= 0:
            raise ValueError("max_wz must be positive and finite")
        if not math.isfinite(cruise_speed) or cruise_speed <= 0:
            raise ValueError("Cruise speed must be positive and finite")
        self.waypoints = route.waypoints
        self.goal_yaw = route.goal_yaw
        self.max_vx = min(float(max_vx), cruise_speed)
        self.max_wz = min(float(max_wz), 0.75)
        self._segment_index = 0
        self._aligning_at_goal = False

    def update(self, pose: Pose) -> Command:
        """Return the next bounded command from one estimated pose."""
        if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
            raise ValueError("Pose must be finite")
        last_segment = len(self.waypoints) - 1
        if last_segment == 0:
            return self._finish_at(pose, self.waypoints[0])

        while self._segment_index < last_segment - 1:
            endpoint = self.waypoints[self._segment_index + 1]
            if math.dist((pose.x, pose.y), endpoint) > self.WAYPOINT_TOLERANCE:
                break
            self._segment_index += 1
        goal = self.waypoints[-1]
        distance = math.dist((pose.x, pose.y), goal)
        if self._segment_index == last_segment - 1 and (
                self._aligning_at_goal or distance <= self.POSITION_TOLERANCE):
            return self._finish_at(pose, goal)

        segment_start = self.waypoints[self._segment_index]
        segment_end = self.waypoints[self._segment_index + 1]
        dx = segment_end[0] - segment_start[0]
        dy = segment_end[1] - segment_start[1]
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            return self._finish_at(pose, segment_end)

        heading = math.atan2(dy, dx)

        cross_track = (dx * (pose.y - segment_start[1])
                       - dy * (pose.x - segment_start[0])) / length
        segment_distance = math.dist((pose.x, pose.y), segment_end)
        approach_speed = min(0.25 * math.sqrt(segment_distance), max(0.035, 0.65 * segment_distance))
        # A higher cruise setting tapers back to the normal approach speed before each waypoint.
        ramp = min(1.0, max(0.0, (segment_distance - self.CRUISE_RAMP_START) / self.CRUISE_RAMP_LENGTH))
        cruise_bonus = max(0.0, self.max_vx - DEFAULT_CRUISE_SPEED)
        speed = min(self.max_vx, approach_speed + ramp * cruise_bonus)
        cross_track_angle = math.atan2(
            self.CROSS_TRACK_GAIN * cross_track,
            max(speed, self.CRUISE_LOOKAHEAD),
        )
        cross_track_angle = _clip(cross_track_angle, self.MAX_CROSS_TRACK_ANGLE)
        desired_heading = heading - cross_track_angle
        remaining = ((segment_end[0] - pose.x) * dx + (segment_end[1] - pose.y) * dy) / length
        if remaining <= 0.08:
            desired_heading = math.atan2(segment_end[1] - pose.y, segment_end[0] - pose.x)
        error = _wrap(desired_heading - pose.yaw)
        if abs(error) > self.ALIGN_THRESHOLD:
            return Command(0.0, _clip(1.8 * error, self.max_wz), "align", False)

        wz = _clip(2.4 * error, self.max_wz)
        vx = speed * max(0.0, math.cos(error))
        return Command(vx, wz, "track", False)

    def _finish_at(self, pose: Pose, goal: tuple[float, float]) -> Command:
        distance = math.dist((pose.x, pose.y), goal)
        # Stretch can drift slightly while turning in place. Keep aligning inside
        # the outer tolerance instead of alternating between translation and yaw.
        tolerance = (self.ALIGNMENT_POSITION_TOLERANCE if self._aligning_at_goal
                     else self.POSITION_TOLERANCE)
        self._aligning_at_goal = distance <= tolerance
        if not self._aligning_at_goal:
            error = _wrap(math.atan2(goal[1] - pose.y, goal[0] - pose.x) - pose.yaw)
            if abs(error) > self.ALIGN_THRESHOLD:
                return Command(0.0, _clip(1.8 * error, self.max_wz), "align", False)
            speed = min(self.max_vx, 0.10, max(0.035, 0.65 * distance))
            return Command(speed, _clip(2.4 * error, self.max_wz), "track", False)

        if self.goal_yaw is not None:
            error = _wrap(self.goal_yaw - pose.yaw)
            if abs(error) > self.HEADING_TOLERANCE:
                return Command(0.0, _clip(1.8 * error, self.max_wz), "final_align", False)
        return Command(0.0, 0.0, "reached", True)


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))
