import math
import json

import pytest

from navigation.docking import Station, station_depth
from navigation.grid_navigation import GridNavigationSession
from navigation.map_exports import rasterize
from navigation.mini_slam.core import LidarModel, SensorSample
from navigation.planning import StaticGrid, plan_route
from navigation.prototype import load_profile
from navigation.tracking import Pose


def test_station_depth_projects_sensor_offset_and_wraps_heading():
    station = Station(Pose(1.0, 0.0, math.pi), 0.65)
    assert station.approach.x == pytest.approx(1.35)
    pose = Pose(1.30, 0.01, -math.pi + 0.07)
    lidar = LidarModel((-0.18, -0.12, -0.04, 0.0, .04, .12, .18), .05, 10.0, (.02, .01))
    origin = .02 * math.cos(.07) - .01 * math.sin(.07)
    ranges = tuple((.95 - origin) / math.cos(angle + .07) for angle in lidar.angles)
    sample = SensorSample((0.0, 0.0), 0.0, (*ranges[:6], 0.22), 1.0)
    assert station_depth(station, pose, lidar, sample) == pytest.approx(.30)
    assert station_depth(station, Pose(1.30, .30, pose.yaw), lidar, sample) is None
    assert station_depth(station, pose, lidar, SensorSample((0., 0.), 0., (None,) * 7, 1.)) is None


def test_trinary_raster_rotates_nothing_and_preserves_unknown():
    grid = StaticGrid(.05, {(-2, 0): -0.62, (-1, 0): 0.61, (-2, 1): 0.62})
    raster = rasterize(grid)
    assert (raster.width, raster.height) == (2, 2)
    assert raster.origin == (-.1, 0.0)
    assert raster.pixels == bytes([0, 205, 254, 205])


def test_kitchen_profile_and_measured_grid_allow_both_station_approaches():
    profile = load_profile()
    radius = profile.footprint_radius + 0.05
    start = profile.initial_pose[:2]
    for label in ("A", "B", "A"):
        station = profile.stations[label]
        approach = (station.approach.x, station.approach.y)
        goal = (station.pose.x, station.pose.y)
        assert plan_route(profile.grid, start, approach, footprint_radius=radius, keepouts=profile.keepouts)
        assert profile.grid.is_segment_clear(approach, goal, footprint_radius=radius, keepouts=profile.keepouts)
        start = goal


def test_pose_only_station_profile_and_navigation_preserve_goal_heading(tmp_path):
    map_path = tmp_path / "free-map.json"
    map_path.write_text(json.dumps({"resolution_m": 0.1,
                                    "slam": [[x, y, -5.0] for x in range(10)
                                             for y in range(10)]}), encoding="utf-8")
    payload = {
        "name": "Pose-only station",
        "scene": "scene.xml",
        "initial_pose": [0.25, 0.25, 0.0],
        "map": str(map_path),
        "footprint_radius_m": 0.1,
        "stations": {"A": {"pose": [0.75, 0.75, math.pi / 2]}},
    }
    profile_path = tmp_path / "profile.json"
    profile_path.write_text(json.dumps(payload), encoding="utf-8")
    profile = load_profile(profile_path)
    station = profile.stations["A"]

    assert not station.uses_lidar_docking
    assert station.pose == Pose(0.75, 0.75, math.pi / 2)

    class _Robot:
        class Data:
            time = 0.0

        data = Data()

    session = GridNavigationSession(_Robot(), profile.grid, pose_source=lambda: Pose(*profile.initial_pose),
                                    radius=profile.footprint_radius)
    followed = []

    def follow(route, *_args):
        followed.append(route)
        session.pose = station.pose

    session._follow = follow
    session._dock = lambda *_args: pytest.fail("pose-only station should not use LiDAR docking")
    visit = session._go(station.pose, deadline=10.0, station=station)

    assert visit.goal == station.pose
    assert len(followed) == 1
    assert followed[0].goal_yaw == math.pi / 2
    assert followed[0].waypoints[-1] == (station.pose.x, station.pose.y)
