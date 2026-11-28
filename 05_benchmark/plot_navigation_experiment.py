"""Plot synchronized navigation estimates and summarize localization errors.

Run after an evaluator has written ``truth.csv``, ``trajectory.csv`` and
``metrics.json`` to one result directory. Matplotlib is supplied by the
project's locked requirements.
"""

import argparse
import csv
import json
import math
from pathlib import Path


ESTIMATORS = {
    "wheel": ("Wheel odometry", "#0072B2", "-"),
    "fused": ("Wheel + IMU", "#D55E00", "--"),
    "slam": ("Mini SLAM", "#009E73", "-."),
}
TRUTH_COLOR = "#222222"


def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _read_csv(path):
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"No samples found in {path}")
    return rows


def _load_run(run_dir):
    run_dir = Path(run_dir)
    truth = _read_csv(run_dir / "truth.csv")
    trajectory = _read_csv(run_dir / "trajectory.csv")
    metrics = json.loads((run_dir / "metrics.json").read_text(encoding="utf-8"))
    if len(truth) != len(trajectory):
        raise ValueError("truth.csv and trajectory.csv must contain the same number of rows")

    times = []
    truth_poses = []
    estimates = {name: [] for name in ESTIMATORS}
    for truth_row, estimate_row in zip(truth, trajectory):
        truth_time = float(truth_row["sim_time_s"])
        estimate_time = float(estimate_row["sim_time_s"])
        if not math.isclose(truth_time, estimate_time, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("truth.csv and trajectory.csv timestamps are not synchronized")
        times.append(truth_time)
        truth_poses.append(tuple(float(truth_row[key]) for key in ("x_m", "y_m", "yaw_rad")))
        for name in ESTIMATORS:
            estimates[name].append(tuple(float(estimate_row[f"{name}_{field}"])
                                         for field in ("x_m", "y_m", "yaw_rad")))
    return times, truth_poses, estimates, metrics


def _errors(truth, estimate):
    positions = [math.hypot(actual[0] - predicted[0], actual[1] - predicted[1])
                 for actual, predicted in zip(truth, estimate)]
    headings = [wrap_angle(predicted[2] - actual[2])
                for actual, predicted in zip(truth, estimate)]
    position_rmse = math.sqrt(sum(error * error for error in positions) / len(positions))
    heading_rmse = math.sqrt(sum(error * error for error in headings) / len(headings))
    terminal_position = positions[-1]
    terminal_heading = abs(headings[-1])
    return positions, headings, position_rmse, heading_rmse, terminal_position, terminal_heading


def _draw_trajectory(path, truth, estimates, metrics):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    figure, axis = plt.subplots(figsize=(9.5, 7.2), constrained_layout=True)
    axis.plot([pose[0] for pose in truth], [pose[1] for pose in truth],
              color=TRUTH_COLOR, linewidth=2.8, label="MuJoCo ground truth", zorder=3)
    for name, (label, color, style) in ESTIMATORS.items():
        points = estimates[name]
        axis.plot([pose[0] for pose in points], [pose[1] for pose in points],
                  color=color, linestyle=style, linewidth=1.7, label=label, zorder=4)

    visits = metrics["visits"]
    for index, visit in enumerate(visits):
        waypoints = visit["path"]
        axis.plot([point[0] for point in waypoints], [point[1] for point in waypoints],
                  color="#666666", linestyle=":", linewidth=1.3, marker=".",
                  markersize=5, label="Planned route" if index == 0 else None, zorder=2)
    start = truth[0]
    axis.scatter(start[0], start[1], marker="o", s=80, color="#56B4E9",
                 edgecolor="black", linewidth=0.7, label="Start", zorder=7)
    if visits:
        goal = visits[-1]["goal"]
        axis.scatter(goal["x"], goal["y"], marker="*", s=150, color="#E69F00",
                     edgecolor="black", linewidth=0.6, label="Goal", zorder=7)

    for index, (xmin, ymin, xmax, ymax) in enumerate(metrics.get("obstacles", [])):
        axis.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin,
                                 facecolor="#BBBBBB", edgecolor="#555555", alpha=0.58,
                                 label="Fixed obstacle" if index == 0 else None, zorder=1))
    axis.set_aspect("equal", adjustable="datalim")
    axis.set_xlabel("World x (m)")
    axis.set_ylabel("World y (m)")
    axis.set_title("Stretch navigation trajectories")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best", frameon=True, fontsize=9)
    figure.savefig(path, dpi=190, facecolor="white")
    plt.close(figure)


def _draw_errors(path, times, errors):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    elapsed = [time - times[0] for time in times]
    figure, (position_axis, heading_axis) = plt.subplots(
        2, 1, figsize=(10, 7), sharex=True, constrained_layout=True)
    for name, (label, color, style) in ESTIMATORS.items():
        positions, headings, *_ = errors[name]
        position_axis.plot(elapsed, [error * 100 for error in positions],
                           color=color, linestyle=style, linewidth=1.7, label=label)
        heading_axis.plot(elapsed, [math.degrees(error) for error in headings],
                          color=color, linestyle=style, linewidth=1.7, label=label)
    position_axis.set_ylabel("Position error (cm)")
    position_axis.set_title("Localization error during one shared run")
    heading_axis.set_ylabel("Heading error (deg)")
    heading_axis.set_xlabel("Elapsed simulation time (s)")
    for axis in (position_axis, heading_axis):
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best", frameon=True, fontsize=9)
    figure.savefig(path, dpi=190, facecolor="white")
    plt.close(figure)


def _summary_text(times, errors, metrics):
    lines = ["# Navigation localization experiment", "",
             f"Samples: {len(times)} at {times[0]:.2f}–{times[-1]:.2f} s simulation time.",
             "All three estimates are compared with MuJoCo base-link ground truth at the same timestamps.",
             "", "| Estimator | Position RMSE (cm) | Heading RMSE (deg) | Final position error (cm) | Final heading error (deg) |",
             "|---|---:|---:|---:|---:|"]
    for name, (label, _, _) in ESTIMATORS.items():
        _, _, pos_rmse, yaw_rmse, terminal_pos, terminal_yaw = errors[name]
        lines.append(f"| {label} | {100 * pos_rmse:.2f} | {math.degrees(yaw_rmse):.2f} | "
                     f"{100 * terminal_pos:.2f} | {math.degrees(terminal_yaw):.2f} |")

    visits = metrics["visits"]
    if visits:
        lines.extend(["", "## Actual goal parking error", "",
                      "These compare the stopped MuJoCo base pose with each configured goal pose.",
                      "", "| Goal | Position error (cm) | Heading error (deg) |", "|---|---:|---:|"])
        for visit in visits:
            distance = visit["position_error_m"]
            heading = visit["heading_error_rad"]
            label = visit["station"]
            lines.append(f"| {label} | {100 * distance:.2f} | {math.degrees(heading):.2f} |")
    lines.extend(["", "## Artifacts", "", "- `trajectories.png` compares the path estimates.",
                  "- `localization_errors.png` shows position and heading errors over simulation time.", ""])
    return "\n".join(lines)


def generate(run_dir, output_dir):
    times, truth, estimates, metrics = _load_run(run_dir)
    errors = {name: _errors(truth, estimate) for name, estimate in estimates.items()}
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    _draw_trajectory(output_dir / "trajectories.png", truth, estimates, metrics)
    _draw_errors(output_dir / "localization_errors.png", times, errors)
    (output_dir / "summary.md").write_text(
        _summary_text(times, errors, metrics), encoding="utf-8")
    return output_dir


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True,
                        help="Directory containing synchronized truth.csv, trajectory.csv and metrics.json")
    parser.add_argument("--out-dir", type=Path,
                        help="Output directory, defaults to RUN_DIR/plots")
    args = parser.parse_args()
    output = args.out_dir or args.run_dir / "plots"
    try:
        result = generate(args.run_dir, output)
    except ModuleNotFoundError as error:
        if error.name == "matplotlib":
            parser.exit(2, "Plotting requires the repository environment from requirements.lock (matplotlib==3.10.1).\n")
        raise
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as error:
        parser.exit(2, f"Could not generate navigation plots: {error}\n")
    print(result)


if __name__ == "__main__":
    main()
