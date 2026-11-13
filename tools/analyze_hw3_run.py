#!/usr/bin/env python3

"""Plot a logged HW3 map/trajectory and compute ground-truth landmark error."""

import argparse
import json
import math
import os

import numpy as np


def load_snapshots(path):
    snapshots = []
    with open(path, "r", encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                snapshots.append(json.loads(line))
    if not snapshots:
        raise ValueError(f"no snapshots found in {path}")
    return snapshots


def load_ground_truth(path):
    with open(path, "r", encoding="utf-8") as stream:
        if path.lower().endswith(".json"):
            data = json.load(stream)
        else:
            try:
                import yaml
            except ImportError as error:
                raise RuntimeError(
                    "PyYAML is required for a YAML ground-truth file"
                ) from error
            data = yaml.safe_load(stream)
    start_pose = data["world_frame"]["robot_start_pose"]
    if any(value is None for value in start_pose):
        raise ValueError("ground-truth robot_start_pose is incomplete")
    tags = {}
    for tag_id, entry in data["tags"].items():
        if entry["x"] is None or entry["y"] is None:
            continue
        tags[int(tag_id)] = np.array([entry["x"], entry["y"]], dtype=float)
    return np.asarray(start_pose, dtype=float), tags


def transform_to_world(points, start_pose):
    points = np.asarray(points, dtype=float)
    cosine = math.cos(start_pose[2])
    sine = math.sin(start_pose[2])
    rotation = np.array([[cosine, -sine], [sine, cosine]], dtype=float)
    return (rotation @ points.T).T + start_pose[:2]


def rotated_covariance(covariance, yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]], dtype=float)
    return rotation @ covariance @ rotation.T


def add_covariance_ellipse(axis, center, covariance, color):
    from matplotlib.patches import Ellipse

    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    order = np.argsort(eigenvalues)[::-1]
    eigenvalues = np.maximum(eigenvalues[order], 0.0)
    eigenvectors = eigenvectors[:, order]
    scale_95 = math.sqrt(5.991464547107979)
    width, height = 2.0 * scale_95 * np.sqrt(eigenvalues)
    angle = math.degrees(math.atan2(eigenvectors[1, 0], eigenvectors[0, 0]))
    axis.add_patch(
        Ellipse(
            center,
            width,
            height,
            angle=angle,
            fill=False,
            edgecolor=color,
            linewidth=1.5,
        )
    )


def analyze(states_path, ground_truth_path, output_directory, title):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise RuntimeError("matplotlib is required for plotting") from error

    snapshots = load_snapshots(states_path)
    start_pose, truth_tags = load_ground_truth(ground_truth_path)
    trajectory_slam = np.array([snapshot["robot"][:2] for snapshot in snapshots])
    trajectory_world = transform_to_world(trajectory_slam, start_pose)
    final = snapshots[-1]

    estimated_tags = {}
    estimated_covariances = {}
    for tag_id_text, entry in final["landmarks"].items():
        tag_id = int(tag_id_text)
        estimated_tags[tag_id] = transform_to_world(
            [entry["position"]], start_pose
        )[0]
        estimated_covariances[tag_id] = rotated_covariance(
            np.asarray(entry["covariance"], dtype=float), start_pose[2]
        )

    errors = {}
    for tag_id in sorted(set(estimated_tags) & set(truth_tags)):
        errors[tag_id] = float(
            np.linalg.norm(estimated_tags[tag_id] - truth_tags[tag_id])
        )
    average_error = None if not errors else float(np.mean(list(errors.values())))
    closure_error = float(np.linalg.norm(trajectory_world[-1] - trajectory_world[0]))

    os.makedirs(output_directory, exist_ok=True)
    metrics = {
        "states_path": os.path.abspath(states_path),
        "mapped_tag_count": len(estimated_tags),
        "matched_ground_truth_tag_count": len(errors),
        "per_tag_error_m": {str(key): value for key, value in errors.items()},
        "average_landmark_error_m": average_error,
        "trajectory_closure_error_m": closure_error,
        "final_robot_covariance_trace": float(
            np.trace(np.asarray(final["robot_covariance"], dtype=float))
        ),
    }
    with open(
        os.path.join(output_directory, "metrics.json"),
        "w",
        encoding="utf-8",
    ) as stream:
        json.dump(metrics, stream, indent=2, sort_keys=True)
        stream.write("\n")

    figure, axis = plt.subplots(figsize=(8, 8))
    axis.plot(
        trajectory_world[:, 0],
        trajectory_world[:, 1],
        color="tab:blue",
        linewidth=1.5,
        label="EKF robot trajectory",
    )
    axis.scatter(
        trajectory_world[0, 0],
        trajectory_world[0, 1],
        marker="o",
        color="green",
        label="start",
        zorder=5,
    )
    for index, (tag_id, position) in enumerate(estimated_tags.items()):
        axis.scatter(
            position[0],
            position[1],
            marker="x",
            color="tab:red",
            label="estimated landmark" if index == 0 else None,
        )
        axis.annotate(
            f"est {tag_id}",
            position,
            xytext=(6, 6),
            textcoords="offset points",
            color="tab:red",
        )
        add_covariance_ellipse(
            axis, position, estimated_covariances[tag_id], "tab:red"
        )
    for index, (tag_id, position) in enumerate(truth_tags.items()):
        axis.scatter(
            position[0],
            position[1],
            marker="+",
            color="black",
            label="ground-truth landmark" if index == 0 else None,
        )
        axis.annotate(
            f"truth {tag_id}",
            position,
            xytext=(6, -14),
            textcoords="offset points",
            color="black",
        )

    axis.set_title(title)
    axis.set_xlabel("world x (m)")
    axis.set_ylabel("world y (m)")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(
        os.path.join(output_directory, "map_trajectory.png"), dpi=200
    )
    plt.close(figure)
    return metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--states", required=True)
    parser.add_argument("--ground-truth", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--title", default="HW3 EKF-SLAM result")
    args = parser.parse_args()
    metrics = analyze(
        args.states, args.ground_truth, args.output_dir, args.title
    )
    print(json.dumps(metrics, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
