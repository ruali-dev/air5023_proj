import math

from navigation.planning import StaticGrid, plan_route
from navigation.tracking import Pose, Route, RouteTracker


def free_grid(width=16, height=16):
    return StaticGrid(0.1, {(x, y): -1.0 for x in range(width) for y in range(height)})


def test_astar_detours_and_straight_routes_use_continuous_clearance():
    grid = free_grid()
    blocked = dict(grid.cells)
    blocked[(7, 7)] = 1.0
    grid = StaticGrid(grid.resolution, blocked)
    start, goal = (0.55, 0.75), (1.05, 0.75)

    route = plan_route(grid, start, goal, footprint_radius=0.0, goal_yaw=math.pi / 2)

    assert route is not None
    assert route.waypoints[0] == start
    assert route.waypoints[-1] == goal
    assert route.goal_yaw == math.pi / 2
    assert len(route.waypoints) > 2
    assert all(grid.is_segment_clear(a, b, footprint_radius=0.0)
               for a, b in zip(route.waypoints, route.waypoints[1:]))


def test_unknown_cells_and_diagonal_corner_cutting_are_blocked():
    grid = free_grid(3, 3)
    cells = dict(grid.cells)
    cells.pop((1, 0))
    cells.pop((0, 1))
    grid = StaticGrid(grid.resolution, cells)

    assert plan_route(grid, (0.05, 0.05), (0.15, 0.15),
                      footprint_radius=0.0) is None
    assert plan_route(grid, (0.05, 0.05), (0.25, 0.25),
                      footprint_radius=0.0) is None


def test_route_tracker_aligns_then_reaches_with_bounded_commands():
    tracker = RouteTracker(Route(((0.0, 0.0), (1.0, 0.0)), math.pi / 2),
                           max_vx=0.5, max_wz=2.0)
    first = tracker.update(Pose(0.0, 0.0, math.pi / 2))
    assert (first.vx, first.phase) == (0.0, "align")
    assert abs(first.wz) <= 0.75

    tracking = tracker.update(Pose(0.0, 0.0, 0.0))
    assert tracking.phase == "track"
    assert 0.0 < tracking.vx <= 0.22

    assert not tracker.update(Pose(1.0, 0.0, 0.0)).reached
    final = tracker.update(Pose(1.0, 0.0, math.pi / 2))
    assert final.reached
    assert (final.vx, final.wz) == (0.0, 0.0)


def test_parking_alignment_allows_small_drift_while_turning():
    tracker = RouteTracker(Route(((0.0, 0.0), (1.0, 0.0)), math.pi / 2),
                           max_vx=0.3, max_wz=1.0)
    assert tracker.update(Pose(.98, 0.0, 0.0)).phase == "final_align"
    assert tracker.update(Pose(.97, 0.0, math.pi / 4)).phase == "final_align"
    assert tracker.update(Pose(.95, 0.0, 0.0)).phase == "track"
    assert tracker.update(Pose(.98, 0.0, math.pi / 2)).reached


def test_faster_cruise_respects_base_limit_and_tapers_before_waypoints():
    route = Route(((0.0, 0.0), (1.0, 0.0)))
    normal = RouteTracker(route, max_vx=0.29, max_wz=1.0)
    faster = RouteTracker(route, max_vx=0.29, max_wz=1.0, cruise_speed=0.30)
    assert normal.update(Pose()).vx == 0.22
    assert faster.update(Pose()).vx == 0.29
    assert faster.update(Pose(.91, 0.0)).vx == normal.update(Pose(.91, 0.0)).vx


def test_experiment_map_supports_detour_and_terminal_approach():
    from navigation.prototype import DEFAULT_PROFILE, load_profile

    profile = load_profile(DEFAULT_PROFILE.parent.parent / "patrol_experiment/profile.json")
    station = profile.stations["A"]
    start = profile.initial_pose[:2]
    approach = (station.approach.x, station.approach.y)
    stop = (station.pose.x, station.pose.y)
    clearance = {"footprint_radius": profile.footprint_radius + 0.05, "keepouts": profile.keepouts}
    assert not profile.grid.is_segment_clear(start, approach, **clearance)
    route = plan_route(profile.grid, start, approach, **clearance)
    assert route is not None and len(route.waypoints) > 2
    assert profile.grid.is_segment_clear(approach, stop, **clearance)


def test_route_respects_rectangular_keepouts():
    grid = free_grid()
    route = plan_route(grid, (0.25, 0.75), (1.25, 0.75), footprint_radius=0.05,
                       keepouts=((0.65, 0.55, 0.85, 0.95),))

    assert route is not None
    assert len(route.waypoints) > 2
    assert all(grid.is_segment_clear(a, b, footprint_radius=0.05,
                                     keepouts=((0.65, 0.55, 0.85, 0.95),))
               for a, b in zip(route.waypoints, route.waypoints[1:]))
