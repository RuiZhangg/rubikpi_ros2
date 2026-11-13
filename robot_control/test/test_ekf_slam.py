import math
import unittest

import numpy as np

from robot_control.ekf_slam import EKFSLAM, wrap_angle


def finite_difference(function, value, epsilon=1e-6):
    value = np.asarray(value, dtype=float)
    baseline = np.asarray(function(value), dtype=float)
    jacobian = np.zeros((baseline.size, value.size), dtype=float)
    for column in range(value.size):
        positive = value.copy()
        negative = value.copy()
        positive[column] += epsilon
        negative[column] -= epsilon
        jacobian[:, column] = (
            np.asarray(function(positive)) - np.asarray(function(negative))
        ) / (2.0 * epsilon)
    return jacobian


class EKFSLAMTest(unittest.TestCase):
    def test_wrap_angle(self):
        self.assertAlmostEqual(wrap_angle(3.0 * math.pi), -math.pi)
        self.assertAlmostEqual(wrap_angle(-0.2), -0.2)

    def test_motion_jacobians_match_finite_difference(self):
        pose = np.array([0.3, -0.2, 0.4])
        control = np.array([0.12, -0.07])
        analytic_state, analytic_control = EKFSLAM.motion_jacobians(
            pose, control
        )
        numeric_state = finite_difference(
            lambda candidate: EKFSLAM.motion_model(candidate, control), pose
        )
        numeric_control = finite_difference(
            lambda candidate: EKFSLAM.motion_model(pose, candidate), control
        )
        np.testing.assert_allclose(analytic_state, numeric_state, atol=1e-7)
        np.testing.assert_allclose(analytic_control, numeric_control, atol=1e-7)

    def test_measurement_jacobian_matches_finite_difference(self):
        slam = EKFSLAM(initial_pose=(0.4, -0.3, 0.5))
        slam.initialize_landmark(7, (1.1, -0.2), np.diag([0.01, 0.02]))
        prediction, analytic = slam.measurement_model(7)

        def model(candidate):
            robot = candidate[:3]
            landmark = candidate[3:5]
            return EKFSLAM._relative_measurement(robot, landmark)

        numeric = finite_difference(model, slam.state)
        self.assertEqual(prediction.shape, (2,))
        np.testing.assert_allclose(analytic, numeric, atol=1e-7)

    def test_dynamic_landmark_augmentation_preserves_cross_covariance(self):
        slam = EKFSLAM(
            initial_pose=(0.2, 0.1, 0.3),
            initial_pose_covariance=np.diag([0.04, 0.03, 0.02]),
        )
        result = slam.observe(4, (1.0, 0.25), np.diag([0.01, 0.01]))
        self.assertTrue(result.initialized)
        self.assertEqual(slam.dimension, 5)
        self.assertEqual(slam.landmark_ids, (4,))
        self.assertGreater(np.linalg.norm(slam.covariance[:3, 3:]), 0.0)
        np.testing.assert_allclose(slam.covariance, slam.covariance.T, atol=1e-12)

    def test_prediction_moves_robot_but_not_landmark_mean(self):
        slam = EKFSLAM()
        slam.observe(0, (1.0, 0.0), np.diag([0.01, 0.01]))
        landmark_before, _ = slam.landmark(0)
        slam.predict(0.05, 0.0, np.diag([0.002**2, 0.01**2]))
        landmark_after, _ = slam.landmark(0)
        np.testing.assert_allclose(landmark_after, landmark_before)
        self.assertAlmostEqual(slam.robot_pose[0], 0.05)

    def test_known_landmark_update_and_outlier_gate(self):
        slam = EKFSLAM(
            initial_pose_covariance=np.diag([0.02, 0.02, 0.01])
        )
        covariance = np.diag([0.02**2, 0.02**2])
        slam.observe(3, (1.0, 0.0), covariance)
        slam.predict(0.10, 0.0, np.diag([0.03**2, 0.03**2]))

        accepted = slam.observe(3, (0.88, 0.01), covariance)
        self.assertTrue(accepted.accepted)
        self.assertFalse(accepted.initialized)
        covariance_before_reject = slam.covariance.copy()
        state_before_reject = slam.state.copy()

        rejected = slam.observe(3, (4.0, 4.0), covariance, gate_chi2=9.21)
        self.assertFalse(rejected.accepted)
        self.assertEqual(rejected.reason, "innovation_gate")
        np.testing.assert_allclose(slam.state, state_before_reject)
        np.testing.assert_allclose(slam.covariance, covariance_before_reject)

    def test_covariance_remains_positive_semidefinite(self):
        slam = EKFSLAM()
        measurement_covariance = np.diag([0.03**2, 0.03**2])
        slam.observe(1, (1.2, 0.3), measurement_covariance)
        for _ in range(12):
            slam.predict(0.05, math.radians(2.0), np.diag([0.02**2, 0.04**2]))
            expected, _ = slam.measurement_model(1)
            slam.observe(1, expected, measurement_covariance)
        eigenvalues = np.linalg.eigvalsh(slam.covariance)
        self.assertGreaterEqual(float(eigenvalues.min()), -1e-10)

    def test_synthetic_square_loop_corrects_biased_motion(self):
        slam = EKFSLAM()
        truth_pose = np.zeros(3, dtype=float)
        truth_landmarks = {
            0: np.array([1.4, -0.3]),
            1: np.array([1.3, 1.4]),
            2: np.array([-0.3, 1.3]),
            3: np.array([-0.4, -0.2]),
        }
        measurement_covariance = np.diag([0.01**2, 0.01**2])
        control_covariance = np.diag([0.02**2, math.radians(2.0) ** 2])

        for tag_id, position in truth_landmarks.items():
            measurement = EKFSLAM._relative_measurement(truth_pose, position)
            slam.observe(tag_id, measurement, measurement_covariance)

        for _edge in range(4):
            for _step in range(20):
                nominal_control = np.array([0.05, 0.0])
                actual_control = np.array([0.049, math.radians(0.05)])
                truth_pose = EKFSLAM.motion_model(truth_pose, actual_control)
                slam.predict(*nominal_control, control_covariance)
                for tag_id, position in truth_landmarks.items():
                    measurement = EKFSLAM._relative_measurement(
                        truth_pose, position
                    )
                    result = slam.observe(
                        tag_id,
                        measurement,
                        measurement_covariance,
                        gate_chi2=25.0,
                    )
                    self.assertTrue(result.accepted)

            nominal_turn = np.array([0.0, math.pi / 2.0])
            actual_turn = np.array([0.0, math.radians(88.5)])
            truth_pose = EKFSLAM.motion_model(truth_pose, actual_turn)
            slam.predict(*nominal_turn, control_covariance)
            for tag_id, position in truth_landmarks.items():
                measurement = EKFSLAM._relative_measurement(
                    truth_pose, position
                )
                slam.observe(
                    tag_id,
                    measurement,
                    measurement_covariance,
                    gate_chi2=100.0,
                )

        position_error = np.linalg.norm(slam.robot_pose[:2] - truth_pose[:2])
        yaw_error = abs(wrap_angle(slam.robot_pose[2] - truth_pose[2]))
        self.assertTrue(np.all(np.isfinite(slam.state)))
        self.assertTrue(np.all(np.isfinite(slam.covariance)))
        self.assertLess(position_error, 0.04)
        self.assertLess(yaw_error, math.radians(2.0))


if __name__ == "__main__":
    unittest.main()
