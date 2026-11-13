import math
import unittest

import numpy as np

from robot_control.landmark_motion import (
    bootstrap_uncertainty,
    estimate_motion,
)


class LandmarkMotionTest(unittest.TestCase):
    def test_recovers_planar_rigid_motion(self):
        yaw = math.radians(3.2)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        rotation = np.array([[cosine, -sine], [sine, cosine]])
        translation = np.array([0.052, -0.003])
        before = {
            0: np.array([1.0, -0.5]),
            1: np.array([1.8, 0.4]),
            2: np.array([0.7, 1.2]),
        }
        after = {
            tag_id: rotation.T @ (position - translation)
            for tag_id, position in before.items()
        }

        estimate = estimate_motion(before, after)

        self.assertAlmostEqual(estimate.forward_m, translation[0], places=10)
        self.assertAlmostEqual(estimate.left_m, translation[1], places=10)
        self.assertAlmostEqual(estimate.delta_yaw_rad, yaw, places=10)
        self.assertLess(estimate.fit_rmse_m, 1e-12)

    def test_requires_two_separated_landmarks(self):
        with self.assertRaisesRegex(ValueError, "at least two"):
            estimate_motion({0: [1.0, 0.0]}, {0: [0.9, 0.0]})
        with self.assertRaisesRegex(ValueError, "baseline"):
            estimate_motion(
                {0: [1.0, 0.0], 1: [1.1, 0.0]},
                {0: [0.9, 0.0], 1: [1.0, 0.0]},
            )

    def test_bootstrap_reports_small_uncertainty_for_stable_samples(self):
        before = {
            0: [[1.0, -0.5], [1.001, -0.5], [0.999, -0.5]],
            1: [[1.8, 0.4], [1.801, 0.4], [1.799, 0.4]],
        }
        after = {
            0: [[0.95, -0.5], [0.951, -0.5], [0.949, -0.5]],
            1: [[1.75, 0.4], [1.751, 0.4], [1.749, 0.4]],
        }

        uncertainty = bootstrap_uncertainty(
            before,
            after,
            minimum_samples=3,
            minimum_baseline_m=0.30,
            resamples=50,
        )

        self.assertLess(uncertainty["forward_std_m"], 0.002)
        self.assertLess(uncertainty["left_std_m"], 0.002)
        self.assertLess(uncertainty["yaw_std_deg"], 0.2)


if __name__ == "__main__":
    unittest.main()
