"""Follow static-grid routes through the Stretch RobotInterface."""

from dataclasses import dataclass
import math

from .docking import station_depth
from .planning import plan_route
from .scene_grid import SceneGrid, build_scene_grid, robot_radius
from .tracking import DEFAULT_CRUISE_SPEED, Pose, Route, RouteTracker


class GridNavigationError(RuntimeError):
    """The planned or commanded movement has insufficient clearance."""


@dataclass(frozen=True)
class Visit:
    goal: Pose
    pose: Pose
    path: tuple[tuple[float, float], ...]
    sim_time: float


class GridNavigationSession:
    """Own navigation commands and pose feedback for one continuous simulator run."""

    def __init__(self, robot, navigation_grid, *, pose_source=None, keepouts=(),
                 radius=None, slam=None, cruise_speed=DEFAULT_CRUISE_SPEED):
        self.robot = robot
        self.grid = navigation_grid
        self.keepouts = tuple(keepouts)
        self.radius = robot_radius(robot.model, robot.data) if radius is None else radius
        self.slam = slam
        self.cruise_speed = cruise_speed
        self._read_pose = pose_source or (slam.read_pose if slam else lambda: _read_pose(robot))
        self.pose = self._read_pose()

    def tick(self):
        self.robot.step()
        self.pose = self._read_pose()
        return self.pose

    def stop(self):
        self.robot.move_base(0.0, 0.0)
        self.robot.server.push_command(self.robot.command)

    def go(self, goal, *, station=None, timeout=240, cancelled=lambda: False,
           on_plan=lambda route: None, on_update=lambda pose, phase: None):
        """Plan and follow a goal. A cancelled job settles and returns None."""
        try:
            if not math.isfinite(timeout) or timeout <= 0:
                raise ValueError("Navigation timeout must be positive and finite")
            return self._go(goal, self.robot.data.time + timeout, station, cancelled, on_plan, on_update)
        finally:
            self.stop()

    def _go(self, goal, deadline, station=None, cancelled=lambda: False,
            on_plan=lambda route: None, on_update=lambda pose, phase: None):
        if station is not None:
            if station.pose != goal:
                raise ValueError("Station pose must match its goal")
            if station.uses_lidar_docking and self.slam is None:
                raise ValueError("LiDAR docking requires Stretch SLAM")
            if not station.uses_lidar_docking:
                station = None
        approach = goal if station is None else station.approach
        start = self.pose
        if station is not None:
            remaining, lateral, heading = station.offsets(start)
            if (-0.05 <= remaining <= station.approach_distance and abs(lateral) <= 0.04
                    and abs(heading) <= 0.12):
                approach = start
        route = plan_route(self.grid, (start.x, start.y), (approach.x, approach.y),
                           footprint_radius=self.radius + 0.05, goal_yaw=approach.yaw,
                           keepouts=self.keepouts)
        if route is None:
            raise GridNavigationError(f"No clear path to goal ({goal.x:g}, {goal.y:g})")
        points = route.waypoints
        if station is not None:
            terminal = (goal.x, goal.y)
            if not self.grid.is_segment_clear(points[-1], terminal,
                                              footprint_radius=self.radius + 0.05,
                                              keepouts=self.keepouts):
                raise GridNavigationError("Station approach lacks clearance")
            points = (*points, terminal)
        on_plan(Route(points, goal.yaw))
        if station is None or approach != start:
            self._follow(route, deadline, cancelled, on_update)
        if station is not None and not cancelled():
            self._dock(station, deadline, cancelled, on_update)
        if cancelled():
            self._settle(on_update, "cancelling")
            return None
        return Visit(goal, self.pose, points, float(self.robot.data.time))

    def _follow(self, route, deadline, cancelled, on_update):
        robot = self.robot
        tracker = RouteTracker(route, max_vx=robot.base_limits[0], max_wz=robot.base_limits[1],
                               cruise_speed=self.cruise_speed)
        while not cancelled() and robot.data.time < deadline:
            command = tracker.update(self.pose)
            if command.reached:
                self._settle(on_update, "settling")
                if tracker.update(self.pose).reached:
                    return
                continue
            self._move(command.vx, command.wz, on_update, command.phase)
        if not cancelled():
            raise TimeoutError("Grid navigation exceeded its simulation-time budget")

    def _dock(self, station, deadline, cancelled, on_update):
        lidar = self.slam.sensors.lidar
        goal = station.pose
        c, s = math.cos(goal.yaw), math.sin(goal.yaw)
        missing = 0
        while not cancelled() and self.robot.data.time < deadline:
            remaining, lateral, heading = station.offsets(self.pose)
            depth = station_depth(station, self.pose, lidar, self.slam.latest_sample)
            if depth is None:
                self.stop()
                on_update(self.tick(), "acquiring station plane")
                missing += 1
                if missing >= math.ceil(0.5 * self.robot.control_hz):
                    raise GridNavigationError("No reliable LiDAR station plane")
                continue
            missing = 0
            corrected = Pose(self.pose.x + (remaining - depth) * c,
                             self.pose.y + (remaining - depth) * s, self.pose.yaw)
            if abs(depth) <= 0.018 and abs(lateral) <= 0.04 and abs(heading) <= 0.04:
                self._settle(on_update, "settling")
                stopped_depth = station_depth(station, self.pose, lidar, self.slam.latest_sample)
                _, stopped_lateral, stopped_heading = station.offsets(self.pose)
                if (stopped_depth is not None and abs(stopped_depth) <= 0.023
                        and abs(stopped_lateral) <= 0.04 and abs(stopped_heading) <= 0.04):
                    return
                continue
            vx = max(-0.04, min(0.085, depth * 0.75)) if abs(depth) > 0.018 else 0.0
            cross_track = math.atan2(lateral, 0.35) if vx > 0 else 0.0
            wz = max(-0.35, min(0.35, -1.8 * (heading + cross_track)))
            self._move(vx, wz, on_update, "docking", corrected=corrected)
        if not cancelled():
            raise TimeoutError("Station docking exceeded its simulation-time budget")

    def _move(self, vx, wz, on_update, phase, *, corrected=None):
        robot = self.robot
        dt = 1 / robot.control_hz
        distance = vx * dt
        radius = self.radius + 0.02 + abs(distance * wz * dt) / 2
        check_poses = (self.pose,) if corrected is None else (self.pose, corrected)
        for pose in check_poses:
            start = (pose.x, pose.y)
            end = (pose.x + distance * math.cos(pose.yaw),
                   pose.y + distance * math.sin(pose.yaw))
            if not self.grid.is_segment_clear(start, end, footprint_radius=radius,
                                              keepouts=self.keepouts):
                raise GridNavigationError("Tracking stopped because the next movement lacks clearance")
        robot.move_base(vx, wz)
        on_update(self.tick(), phase)

    def _settle(self, on_update, phase):
        self.stop()
        quiet = 0
        hz = self.robot.control_hz
        previous = self.slam.latest_sample if self.slam else None
        for tick in range(math.ceil(2 * hz)):
            on_update(self.tick(), phase)
            if self.slam is None:
                quiet = 3
            else:
                sample = self.slam.latest_sample
                dt = sample.sim_time - previous.sim_time
                wheel_speed = max(abs(a - b) for a, b in zip(sample.wheel_pos, previous.wheel_pos))
                still = (wheel_speed * self.slam.sensors.wheel_radius < 0.015 * dt
                         and abs(sample.gyro_z) < 0.04)
                quiet = quiet + 1 if still else 0
                previous = sample
            if tick + 1 >= math.ceil(0.3 * hz) and quiet >= 3:
                return
        raise GridNavigationError("The base did not settle after a zero-velocity command")


def navigate_grid(robot, waypoints, timeout=150, *, on_arrival=None, pose_source=None,
                  navigation_grid=None):
    """Visit world-frame goals on a frozen grid using one pose source."""
    try:
        goals = tuple(Pose(*map(float, waypoint)) for waypoint in waypoints)
        if any(not all(math.isfinite(v) for v in (goal.x, goal.y, goal.yaw)) for goal in goals):
            raise ValueError("Waypoints must contain finite world x,y,yaw")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("Navigation timeout must be positive and finite")
        if not goals:
            return []

        read_pose = pose_source if pose_source is not None else lambda: _read_pose(robot)
        start = read_pose()
        if navigation_grid is None:
            points = [(point.x, point.y) for point in (start, *goals)]
            scene = build_scene_grid(robot.model, robot.data, points)
        else:
            scene = SceneGrid(navigation_grid, robot_radius(robot.model, robot.data))
        deadline = robot.data.time + timeout
        session = GridNavigationSession(robot, scene.grid, pose_source=read_pose, radius=scene.robot_radius)
        visits = []
        for goal in goals:
            if robot.data.time >= deadline:
                raise TimeoutError("Grid navigation exceeded its simulation-time budget")
            session.pose = read_pose()
            visit = session._go(goal, deadline)
            visits.append(visit)
            if on_arrival is not None:
                on_arrival(visit)
        return visits
    finally:
        robot.move_base(0.0, 0.0)
        robot.server.push_command(robot.command)


def _read_pose(robot):
    return Pose(*map(float, robot.get_state()["base_pose"]))
