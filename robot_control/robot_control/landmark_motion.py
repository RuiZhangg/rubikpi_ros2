"""Planar rover-motion estimation from common before/after landmarks."""

from dataclasses import dataclass
import math
from typing import Dict, Iterable, Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class LandmarkMotionEstimate:
    tag_ids: tuple
    forward_m: float
    left_m: float
    distance_m: float
    delta_yaw_rad: float
    fit_rmse_m: float
    fit_max_residual_m: float
    landmark_baseline_m: float

    def as_dict(self):
        return {
            "tag_ids": list(self.tag_ids),
            "forward_m": self.forward_m,
            "left_m": self.left_m,
            "distance_m": self.distance_m,
            "delta_yaw_rad": self.delta_yaw_rad,
            "delta_yaw_deg": math.degrees(self.delta_yaw_rad),
            "fit_rmse_m": self.fit_rmse_m,
            "fit_max_residual_m": self.fit_max_residual_m,
            "landmark_baseline_m": self.landmark_baseline_m,
        }


def robust_centers(
    samples_by_tag: Mapping[int, Iterable[Sequence[float]]],
    minimum_samples: int,
) -> Dict[int, np.ndarray]:
    centers = {}
    for tag_id, samples in samples_by_tag.items():
        array = np.asarray(list(samples), dtype=float)
        if array.ndim != 2 or array.shape[1] != 2:
            raise ValueError("each landmark sample must contain forward and left")
        if len(array) >= minimum_samples:
            centers[int(tag_id)] = np.median(array, axis=0)
    return centers


def _maximum_baseline(points: np.ndarray) -> float:
    maximum = 0.0
    for first in range(len(points)):
        for second in range(first + 1, len(points)):
            maximum = max(
                maximum,
                float(np.linalg.norm(points[first] - points[second])),
            )
    return maximum


def estimate_motion(
    before_centers: Mapping[int, Sequence[float]],
    after_centers: Mapping[int, Sequence[float]],
    minimum_baseline_m: float = 0.30,
) -> LandmarkMotionEstimate:
    """Fit z_before = R(delta_yaw) z_after + translation."""
    tag_ids = tuple(sorted(set(before_centers) & set(after_centers)))
    if len(tag_ids) < 2:
        raise ValueError("at least two common landmarks are required")

    before = np.asarray([before_centers[tag_id] for tag_id in tag_ids], dtype=float)
    after = np.asarray([after_centers[tag_id] for tag_id in tag_ids], dtype=float)
    if before.shape != after.shape or before.shape[1] != 2:
        raise ValueError("landmark centers must be finite planar coordinates")
    if not np.all(np.isfinite(before)) or not np.all(np.isfinite(after)):
        raise ValueError("landmark centers contain non-finite values")

    baseline = min(_maximum_baseline(before), _maximum_baseline(after))
    if baseline < float(minimum_baseline_m):
        raise ValueError(
            f"landmark baseline {baseline:.3f} m is below "
            f"the required {minimum_baseline_m:.3f} m"
        )

    before_mean = np.mean(before, axis=0)
    after_mean = np.mean(after, axis=0)
    before_centered = before - before_mean
    after_centered = after - after_mean
    cross_covariance = after_centered.T @ before_centered
    left_vectors, _, right_vectors_transposed = np.linalg.svd(cross_covariance)
    rotation = right_vectors_transposed.T @ left_vectors.T
    if np.linalg.det(rotation) < 0.0:
        right_vectors_transposed[-1, :] *= -1.0
        rotation = right_vectors_transposed.T @ left_vectors.T

    translation = before_mean - rotation @ after_mean
    fitted = (rotation @ after.T).T + translation
    residual_norms = np.linalg.norm(before - fitted, axis=1)
    delta_yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return LandmarkMotionEstimate(
        tag_ids=tag_ids,
        forward_m=float(translation[0]),
        left_m=float(translation[1]),
        distance_m=float(np.linalg.norm(translation)),
        delta_yaw_rad=float(delta_yaw),
        fit_rmse_m=float(np.sqrt(np.mean(residual_norms**2))),
        fit_max_residual_m=float(np.max(residual_norms)),
        landmark_baseline_m=baseline,
    )


def bootstrap_uncertainty(
    before_samples: Mapping[int, Iterable[Sequence[float]]],
    after_samples: Mapping[int, Iterable[Sequence[float]]],
    minimum_samples: int,
    minimum_baseline_m: float,
    resamples: int = 300,
    seed: int = 276,
):
    common = sorted(set(before_samples) & set(after_samples))
    arrays_before = {
        tag_id: np.asarray(list(before_samples[tag_id]), dtype=float)
        for tag_id in common
    }
    arrays_after = {
        tag_id: np.asarray(list(after_samples[tag_id]), dtype=float)
        for tag_id in common
    }
    common = [
        tag_id
        for tag_id in common
        if len(arrays_before[tag_id]) >= minimum_samples
        and len(arrays_after[tag_id]) >= minimum_samples
    ]
    if len(common) < 2:
        raise ValueError("bootstrap requires two sufficiently sampled landmarks")

    random = np.random.default_rng(seed)
    estimates = []
    for _ in range(int(resamples)):
        before_centers = {}
        after_centers = {}
        for tag_id in common:
            before = arrays_before[tag_id]
            after = arrays_after[tag_id]
            before_resample = before[random.integers(0, len(before), len(before))]
            after_resample = after[random.integers(0, len(after), len(after))]
            before_centers[tag_id] = np.median(before_resample, axis=0)
            after_centers[tag_id] = np.median(after_resample, axis=0)
        estimate = estimate_motion(
            before_centers,
            after_centers,
            minimum_baseline_m=minimum_baseline_m,
        )
        estimates.append(
            [estimate.forward_m, estimate.left_m, estimate.delta_yaw_rad]
        )

    values = np.asarray(estimates, dtype=float)
    values[:, 2] = np.unwrap(values[:, 2])
    standard_deviation = np.std(values, axis=0, ddof=1)
    return {
        "forward_std_m": float(standard_deviation[0]),
        "left_std_m": float(standard_deviation[1]),
        "yaw_std_rad": float(standard_deviation[2]),
        "yaw_std_deg": float(math.degrees(standard_deviation[2])),
        "resamples": int(resamples),
    }
