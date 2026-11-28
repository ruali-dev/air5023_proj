"""TEAM-01: run waypoint or grid navigation without grasping."""
import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from project_bootstrap import activate
ROOT = activate()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--scene", type=Path, help="alternate scene for grid runs with explicit goals")
    parser.add_argument("--planner", choices=("waypoints", "grid"), default="waypoints")
    parser.add_argument("--localization", choices=("sim", "slam"), default="sim",
                        help="Grid feedback from simulator pose or Stretch Mini SLAM")
    parser.add_argument("--initial-pose", type=float, nargs=3, metavar=("X", "Y", "YAW"),
                        help="Known initial SLAM pose, defaulting to the scene spawn")
    maps = parser.add_mutually_exclusive_group()
    maps.add_argument("--map-in", type=Path, help="Plan and localize on a saved Mini SLAM map")
    maps.add_argument("--map-out", type=Path, help="Save the occupancy map built during the route")
    parser.add_argument("--export-map", type=Path, help="Export the loaded or built map as PGM and YAML")
    parser.add_argument("--goal", type=float, action="append", nargs=3, metavar=("X", "Y", "YAW"),
                        help="World-frame grid goal in metres and radians. Repeat for a patrol.")
    parser.add_argument("--timeout", type=float, help="Simulated seconds (default 45 for waypoints, 150 for grid)")
    parser.add_argument("--capture-dir", type=Path, help="Save one head RGB image at each grid goal")
    args = parser.parse_args()
    if args.timeout is not None and (not math.isfinite(args.timeout) or args.timeout <= 0):
        parser.error("--timeout must be positive and finite")
    timeout = args.timeout if args.timeout is not None else (150 if args.planner == "grid" else 45)
    if args.planner != "grid" and (args.goal or args.capture_dir):
        parser.error("--goal and --capture-dir require --planner grid")
    if args.scene and (args.planner != "grid" or not args.goal):
        parser.error("--scene requires --planner grid and explicit --goal values")
    if args.scene and args.localization == "slam" and args.initial_pose is None:
        parser.error("SLAM on an alternate --scene requires --initial-pose")
    if args.localization == "slam" and args.planner != "grid":
        parser.error("--localization slam requires --planner grid")
    if args.localization != "slam" and (args.initial_pose or args.map_in or args.map_out):
        parser.error("--initial-pose, --map-in and --map-out require --localization slam")
    if args.initial_pose and not all(math.isfinite(value) for value in args.initial_pose):
        parser.error("--initial-pose must contain finite coordinates and yaw")
    if args.map_out is not None and args.map_out.exists():
        parser.error("--map-out must name a new file")
    if args.export_map and not (args.map_in or args.map_out):
        parser.error("--export-map requires --map-in or --map-out")
    if args.capture_dir is not None:
        args.capture_dir.mkdir(parents=True, exist_ok=True)
        if any(args.capture_dir.iterdir()):
            parser.error("--capture-dir must be empty")
    if args.headless:
        os.environ.setdefault("MUJOCO_GL", "glfw" if sys.platform == "win32" else "egl")
    from robot.robot_interface import RobotInterface
    from navigation.navigation_interface import navigate
    scene = args.scene or ROOT / "scenes/kitchen/generated/kitchen.xml"
    metadata = {} if args.scene else json.loads(scene.with_name("metadata.json").read_text())
    reference_grid = None
    if args.localization == "slam":
        from navigation.slam_navigation import StretchSLAM, load_map, save_map

        if args.map_in:
            try:
                reference_grid = load_map(args.map_in)
            except (OSError, ValueError) as error:
                parser.exit(1, f"Mini SLAM: {error}\n")
    with RobotInterface(scene, gui=not args.headless, realtime=not args.headless,
                        cameras=args.capture_dir is not None) as robot:
        robot.move_joints({"lift": .55, "arm": 0.0}, 2.5)
        if robot.viewer:
            from perception.camera import configure_overview
            configure_overview(robot.viewer.cam)
        if args.planner == "grid":
            from navigation.grid_navigation import GridNavigationError, navigate_grid

            captures = []

            def capture(_visit):
                from PIL import Image

                output = args.capture_dir / f"head-{len(captures) + 1:03d}.png"
                Image.fromarray(robot.get_camera_image("head")).save(output)
                captures.append(str(output))

            try:
                slam = None
                pose_source = None
                if args.localization == "slam":
                    initial_pose = args.initial_pose
                    if initial_pose is None:
                        x, y, _height = metadata["robot_start"]
                        initial_pose = (x, y, 0.0)
                    slam = StretchSLAM(robot, initial_pose, reference_grid=reference_grid)
                    pose_source = slam.read_pose
                visits = navigate_grid(robot, args.goal or metadata["navigation_waypoints"],
                                       timeout=timeout, pose_source=pose_source,
                                       navigation_grid=reference_grid,
                                       on_arrival=capture if args.capture_dir else None)
                result = {"visits": [asdict(visit) for visit in visits], "images": captures}
                if slam is not None:
                    estimator = slam.estimator
                    result["slam"] = {"matches_attempted": estimator.matches_attempted,
                                      "matches_accepted": estimator.matches_accepted,
                                      "mapped_cells": len(estimator.grid.cells)}
                    if args.map_out:
                        save_map(estimator.grid, args.map_out)
                        result["slam"]["map_path"] = str(args.map_out)
                    if args.export_map:
                        from navigation.map_exports import export_map

                        export_map(reference_grid or load_map(args.map_out), args.export_map)
            except (GridNavigationError, TimeoutError, OSError, ValueError) as error:
                parser.exit(1, f"Grid navigation: {error}\n")
            print(json.dumps(result))
        else:
            navigate(robot, metadata["navigation_waypoints"], timeout=timeout)
            print(json.dumps({"base_pose": robot.get_state()["base_pose"].tolist(),
                              "simulation_time": robot.data.time}))


if __name__ == "__main__":
    main()
