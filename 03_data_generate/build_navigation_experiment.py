"""Add a stationary obstacle and a new Stretch spawn to a copy of the kitchen."""

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
KITCHEN = ROOT / "scenes/kitchen/generated/kitchen.xml"
PROFILE = ROOT / "01_navigation/maps/patrol_experiment/profile.json"
OBSTACLE_HEIGHT_M = 0.38


def build(base_scene, profile_path):
    import mujoco

    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    output = (profile_path.parent / profile["scene"]).resolve()
    if output in (base_scene.resolve(), KITCHEN.resolve()):
        raise ValueError("The experiment must have a separate output scene")
    scene = ET.parse(base_scene).getroot()
    # Keep asset references relative to the base scene even if output is elsewhere.
    compiler = scene.find("compiler")
    for asset in scene.find("asset"):
        filename = asset.get("file")
        if filename and not Path(filename).is_absolute():
            directory = "meshdir" if asset.tag == "mesh" else "texturedir"
            asset_dir = compiler.get(directory, compiler.get("assetdir", ""))
            asset.set("file", (base_scene.parent / asset_dir / filename).resolve().as_posix())

    world = scene.find("worldbody")
    base = world.find("body[@name='base_link']")
    x, y, yaw = profile["initial_pose"]
    height = base.get("pos", "0 0 0").split()[2]
    base.set("pos", f"{x} {y} {height}")
    base.set("quat", f"{math.cos(yaw / 2)} 0 0 {math.sin(yaw / 2)}")
    keyframe = scene.find("keyframe")
    if keyframe is not None:
        scene.remove(keyframe)  # The original keys encode the old spawn.
    scene.set("model", "stretch_kitchen_navigation_experiment")
    camera_x, camera_y, camera_z = profile.get("overview_camera", (0.18, -2.48, 6.2))
    ET.SubElement(world, "camera", name="navigation_overview",
                  pos=f"{camera_x} {camera_y} {camera_z}", xyaxes="1 0 0 0 1 0", fovy="36")

    names = []
    for index, (xmin, ymin, xmax, ymax) in enumerate(profile["obstacles"], start=1):
        if not all(map(math.isfinite, (xmin, ymin, xmax, ymax))) or xmin >= xmax or ymin >= ymax:
            raise ValueError("Obstacle bounds must describe a finite non-empty rectangle")
        name = f"navigation_obstacle_{index}"
        names.append(name)
        z = OBSTACLE_HEIGHT_M / 2
        body = ET.SubElement(world, "body", name=name, pos=f"{(xmin + xmax) / 2} {(ymin + ymax) / 2} {z}")
        ET.SubElement(body, "geom", name=name, type="box", size=f"{(xmax - xmin) / 2} {(ymax - ymin) / 2} {z}",
                      rgba="0.86 0.34 0.14 1", contype="1", conaffinity="1")

    xml = ET.tostring(scene, encoding="unicode")
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    for name in names:
        obstacle = model.geom(name).id
        for other in range(model.ngeom):
            if other == obstacle or not model.geom_contype[other] or "floor" in model.geom(other).name:
                continue
            if mujoco.mj_geomDistance(model, data, obstacle, other, 0.001, None) < -0.001:
                raise ValueError(f"{name} overlaps {model.geom(other).name or model.geom_bodyid[other]}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(xml + "\n", encoding="utf-8")
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-scene", type=Path, default=KITCHEN)
    parser.add_argument("--profile", type=Path, default=PROFILE)
    args = parser.parse_args()
    print(build(args.base_scene, args.profile))


if __name__ == "__main__":
    main()
