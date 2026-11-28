"""Scene and map configuration for the live Navigation Prototype."""

from contextlib import contextmanager
from dataclasses import dataclass
import json
import math
from pathlib import Path

from .docking import Station
from .grid_navigation import GridNavigationSession
from .planning import StaticGrid
from .slam_navigation import StretchSLAM, load_map
from .tracking import DEFAULT_CRUISE_SPEED, Pose


DEFAULT_PROFILE = Path(__file__).resolve().parents[1] / "maps" / "kitchen" / "profile.json"


@dataclass(frozen=True)
class NavigationProfile:
    name: str
    scene: Path
    initial_pose: tuple[float, float, float]
    grid: StaticGrid
    footprint_radius: float
    stations: dict[str, Station]
    keepouts: tuple[tuple[float, float, float, float], ...]
    cruise_speed: float = DEFAULT_CRUISE_SPEED
    obstacles: tuple[tuple[float, float, float, float], ...] = ()


def load_profile(path=None):
    path = Path(path or DEFAULT_PROFILE).resolve()
    try:
        config = json.loads(path.read_text(encoding="utf-8"))
        initial_pose = tuple(map(float, config["initial_pose"]))
        if len(initial_pose) != 3 or not all(math.isfinite(value) for value in initial_pose):
            raise ValueError("Expected a finite x, y, yaw initial pose")
        footprint_radius = float(config["footprint_radius_m"])
        if not 0 < footprint_radius < math.inf:
            raise ValueError("Expected a positive finite footprint radius")
        cruise_speed = float(config.get("cruise_speed_m_s", DEFAULT_CRUISE_SPEED))
        if not 0 < cruise_speed < math.inf:
            raise ValueError("Expected a positive finite cruise speed")
        keepouts = tuple(tuple(map(float, bounds)) for bounds in config.get("keepouts", ()))
        obstacles = tuple(tuple(map(float, bounds)) for bounds in config.get("obstacles", ()))
        StaticGrid._validate_clearance(0.0, keepouts + obstacles)
        stations = {
            name: Station(Pose(*map(float, entry["pose"])), float(entry["stand_range"]),
                          float(entry.get("approach_distance", 0.35)))
            for name, entry in config["stations"].items()
        }
        if stations.keys() != {"A", "B"}:
            raise ValueError("Expected station definitions for A and B")
        return NavigationProfile(config["name"], (path.parent / config["scene"]).resolve(),
                                 initial_pose, load_map(path.parent / config["map"]),
                                 footprint_radius, stations, keepouts + obstacles, cruise_speed, obstacles)
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"Invalid navigation profile {path}: {error}") from error


@contextmanager
def open_session(profile, *, viewer=False, realtime=True, cameras=False):
    """Open one Stretch world with known spawn and a frozen sensor map."""
    from robot.robot_interface import RobotInterface

    if not profile.scene.is_file():
        raise FileNotFoundError(f"Missing scene {profile.scene}. Generate the profile's world first.")
    with RobotInterface(profile.scene, gui=viewer, realtime=realtime, cameras=cameras,
                        sample_cameras_on_step=False) as robot:
        robot.move_joints({"lift": 0.55, "arm": 0.0}, 2.5)
        if robot.viewer:
            from perception.camera import configure_overview

            configure_overview(robot.viewer.cam)
        slam = StretchSLAM(robot, profile.initial_pose, reference_grid=profile.grid)
        session = GridNavigationSession(robot, profile.grid, keepouts=profile.keepouts,
                                        radius=profile.footprint_radius, slam=slam,
                                        cruise_speed=profile.cruise_speed)
        try:
            yield session
        finally:
            session.stop()
