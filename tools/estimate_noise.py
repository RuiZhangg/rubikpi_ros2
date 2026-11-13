#!/usr/bin/env python3

"""Estimate preliminary EKF noise parameters from arena CSV measurements."""

import argparse
import csv
import json
import math

import numpy as np


def read_numeric_rows(path):
    with open(path, "r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def estimate_motion(path):
    rows = read_numeric_rows(path)
    errors = []
    for row in rows:
        distance_error = float(row["actual_distance_m"]) - float(
            row["commanded_distance_m"]
        )
        yaw_error = math.atan2(
            math.sin(
                float(row["actual_delta_yaw_rad"])
                - float(row["commanded_delta_yaw_rad"])
            ),
            math.cos(
                float(row["actual_delta_yaw_rad"])
                - float(row["commanded_delta_yaw_rad"])
            ),
        )
        errors.append([distance_error, yaw_error])
    if len(errors) < 2:
        raise ValueError("motion calibration requires at least two trials")
    errors = np.asarray(errors, dtype=float)
    return {
        "trial_count": int(len(errors)),
        "mean_error": np.mean(errors, axis=0).tolist(),
        "sample_covariance": np.cov(errors, rowvar=False, ddof=1).tolist(),
        "sample_standard_deviation": np.std(errors, axis=0, ddof=1).tolist(),
    }


def estimate_measurement(path):
    rows = read_numeric_rows(path)
    samples_by_tag = {}
    for row in rows:
        if row.get("accepted", "1") not in ("1", "true", "True"):
            continue
        tag_id = int(row["tag_id"])
        samples_by_tag.setdefault(tag_id, []).append(
            [float(row["z_forward_m"]), float(row["z_left_m"])]
        )
    summaries = {}
    pooled_residuals = []
    for tag_id, samples in sorted(samples_by_tag.items()):
        samples = np.asarray(samples, dtype=float)
        if len(samples) < 2:
            continue
        mean = np.mean(samples, axis=0)
        residuals = samples - mean
        pooled_residuals.extend(residuals.tolist())
        summaries[str(tag_id)] = {
            "sample_count": int(len(samples)),
            "mean": mean.tolist(),
            "sample_covariance": np.cov(samples, rowvar=False, ddof=1).tolist(),
        }
    if len(pooled_residuals) < 2:
        raise ValueError("measurement calibration needs repeated accepted samples")
    pooled_residuals = np.asarray(pooled_residuals, dtype=float)
    return {
        "tag_summaries": summaries,
        "pooled_sample_count": int(len(pooled_residuals)),
        "pooled_covariance": np.cov(
            pooled_residuals, rowvar=False, ddof=1
        ).tolist(),
        "pooled_standard_deviation": np.std(
            pooled_residuals, axis=0, ddof=1
        ).tolist(),
    }


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="mode", required=True)
    motion_parser = subparsers.add_parser("motion")
    motion_parser.add_argument("csv")
    measurement_parser = subparsers.add_parser("measurement")
    measurement_parser.add_argument("csv")
    args = parser.parse_args()

    if args.mode == "motion":
        result = estimate_motion(args.csv)
    else:
        result = estimate_measurement(args.csv)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
