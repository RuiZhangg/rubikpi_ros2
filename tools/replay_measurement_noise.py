#!/usr/bin/env python3

"""Replay stationary observation CSVs against a candidate EKF noise model."""

import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "robot_control"))

from robot_control.ekf_slam import EKFSLAM  # noqa: E402


def replay(paths, sigma_base, sigma_per_meter, gate_chi2):
    accepted = 0
    rejected = 0
    observation_count = 0
    nis_values = []
    worst = None

    for path_value in paths:
        path = Path(path_value)
        filter_instance = EKFSLAM(
            initial_pose_covariance=np.diag([1e-8, 1e-8, 1e-8])
        )
        with path.open("r", encoding="utf-8", newline="") as stream:
            for row in csv.DictReader(stream):
                if not row.get("z_forward_m") or not row.get("z_left_m"):
                    continue
                range_m = float(row["range_m"])
                sigma = float(sigma_base) + float(sigma_per_meter) * range_m
                covariance = np.diag([sigma**2, sigma**2])
                result = filter_instance.observe(
                    int(row["tag_id"]),
                    [float(row["z_forward_m"]), float(row["z_left_m"])],
                    covariance,
                    gate_chi2=float(gate_chi2),
                )
                observation_count += 1
                if result.accepted:
                    accepted += 1
                else:
                    rejected += 1
                if result.nis is not None:
                    nis_values.append(result.nis)
                    if worst is None or result.nis > worst["nis"]:
                        worst = {
                            "nis": result.nis,
                            "csv": str(path),
                            "tag_id": result.tag_id,
                            "range_m": range_m,
                            "innovation": list(result.innovation),
                        }

    if not nis_values:
        raise ValueError("replay requires at least one repeated observation")
    nis_array = np.asarray(nis_values, dtype=float)
    return {
        "measurement_sigma_base_m": float(sigma_base),
        "measurement_sigma_per_meter": float(sigma_per_meter),
        "gate_chi2": float(gate_chi2),
        "observation_count": observation_count,
        "accepted": accepted,
        "rejected": rejected,
        "nis_p50": float(np.percentile(nis_array, 50)),
        "nis_p95": float(np.percentile(nis_array, 95)),
        "nis_p99": float(np.percentile(nis_array, 99)),
        "nis_max": float(np.max(nis_array)),
        "worst_observation": worst,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="+")
    parser.add_argument("--sigma-base", type=float, required=True)
    parser.add_argument("--sigma-per-meter", type=float, required=True)
    parser.add_argument("--gate-chi2", type=float, default=9.21034037197618)
    args = parser.parse_args()
    result = replay(
        args.csv,
        args.sigma_base,
        args.sigma_per_meter,
        args.gate_chi2,
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
