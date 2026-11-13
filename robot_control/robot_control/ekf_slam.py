"""ROS-independent planar EKF-SLAM implementation for CSE 276A HW3."""

from dataclasses import dataclass
import math
from typing import Dict, Iterable, Optional, Tuple

import numpy as np


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _as_covariance(value: Iterable[Iterable[float]], size: int) -> np.ndarray:
    covariance = np.asarray(value, dtype=float)
    if covariance.shape != (size, size):
        raise ValueError(f"expected a {size}x{size} covariance")
    if not np.all(np.isfinite(covariance)):
        raise ValueError("covariance contains a non-finite value")
    covariance = 0.5 * (covariance + covariance.T)
    if np.min(np.linalg.eigvalsh(covariance)) < -1e-10:
        raise ValueError("covariance must be positive semidefinite")
    return covariance


@dataclass(frozen=True)
class ObservationResult:
    """Outcome of one landmark observation."""

    tag_id: int
    accepted: bool
    initialized: bool
    innovation: Tuple[float, float]
    nis: Optional[float]
    reason: str


class EKFSLAM:
    """Planar EKF-SLAM with a dynamic [robot, landmark positions] state."""

    def __init__(
        self,
        initial_pose: Iterable[float] = (0.0, 0.0, 0.0),
        initial_pose_covariance: Iterable[Iterable[float]] = np.diag(
            [1e-8, 1e-8, 1e-8]
        ),
    ) -> None:
        pose = np.asarray(tuple(initial_pose), dtype=float)
        if pose.shape != (3,) or not np.all(np.isfinite(pose)):
            raise ValueError("initial_pose must contain three finite values")

        self.state = pose.copy()
        self.state[2] = wrap_angle(self.state[2])
        self.covariance = _as_covariance(initial_pose_covariance, 3)
        self._landmark_slices: Dict[int, slice] = {}

    @property
    def dimension(self) -> int:
        return int(self.state.size)

    @property
    def landmark_ids(self) -> Tuple[int, ...]:
        return tuple(self._landmark_slices.keys())

    @property
    def robot_pose(self) -> np.ndarray:
        return self.state[:3].copy()

    @property
    def robot_covariance(self) -> np.ndarray:
        return self.covariance[:3, :3].copy()

    def landmark(self, tag_id: int) -> Tuple[np.ndarray, np.ndarray]:
        landmark_slice = self._landmark_slices[int(tag_id)]
        position = self.state[landmark_slice].copy()
        covariance = self.covariance[landmark_slice, landmark_slice].copy()
        return position, covariance

    @staticmethod
    def motion_model(pose: np.ndarray, control: np.ndarray) -> np.ndarray:
        distance, delta_yaw = control
        heading_midpoint = pose[2] + 0.5 * delta_yaw
        return np.array(
            [
                pose[0] + distance * math.cos(heading_midpoint),
                pose[1] + distance * math.sin(heading_midpoint),
                wrap_angle(pose[2] + delta_yaw),
            ],
            dtype=float,
        )

    @staticmethod
    def motion_jacobians(
        pose: np.ndarray,
        control: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        distance, delta_yaw = control
        heading_midpoint = pose[2] + 0.5 * delta_yaw
        cosine = math.cos(heading_midpoint)
        sine = math.sin(heading_midpoint)

        state_jacobian = np.array(
            [
                [1.0, 0.0, -distance * sine],
                [0.0, 1.0, distance * cosine],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        control_jacobian = np.array(
            [
                [cosine, -0.5 * distance * sine],
                [sine, 0.5 * distance * cosine],
                [0.0, 1.0],
            ],
            dtype=float,
        )
        return state_jacobian, control_jacobian

    def predict(
        self,
        distance: float,
        delta_yaw: float,
        control_covariance: Iterable[Iterable[float]],
    ) -> None:
        """Apply one completed commanded-motion primitive."""
        control = np.array([distance, delta_yaw], dtype=float)
        if not np.all(np.isfinite(control)):
            raise ValueError("control contains a non-finite value")
        motion_covariance = _as_covariance(control_covariance, 2)

        robot_jacobian, robot_control_jacobian = self.motion_jacobians(
            self.state[:3], control
        )
        full_state_jacobian = np.eye(self.dimension, dtype=float)
        full_state_jacobian[:3, :3] = robot_jacobian
        full_control_jacobian = np.zeros((self.dimension, 2), dtype=float)
        full_control_jacobian[:3, :] = robot_control_jacobian

        self.state[:3] = self.motion_model(self.state[:3], control)
        # Some Accelerate/NumPy combinations emit spurious floating-point
        # warnings from finite matrix products. Check the result explicitly so
        # a real numerical failure still stops the filter.
        with np.errstate(over="ignore", divide="ignore", invalid="ignore"):
            predicted_covariance = (
                full_state_jacobian
                @ self.covariance
                @ full_state_jacobian.T
                + full_control_jacobian
                @ motion_covariance
                @ full_control_jacobian.T
            )
        if not np.all(np.isfinite(predicted_covariance)):
            raise FloatingPointError("non-finite covariance after prediction")
        self.covariance = predicted_covariance
        self._stabilize_covariance()

    @staticmethod
    def _relative_measurement(
        robot_pose: np.ndarray,
        landmark_position: np.ndarray,
    ) -> np.ndarray:
        delta_x = landmark_position[0] - robot_pose[0]
        delta_y = landmark_position[1] - robot_pose[1]
        cosine = math.cos(robot_pose[2])
        sine = math.sin(robot_pose[2])
        return np.array(
            [
                cosine * delta_x + sine * delta_y,
                -sine * delta_x + cosine * delta_y,
            ],
            dtype=float,
        )

    def measurement_model(self, tag_id: int) -> Tuple[np.ndarray, np.ndarray]:
        """Return predicted robot-frame tag center and full Jacobian H."""
        landmark_slice = self._landmark_slices[int(tag_id)]
        prediction = self._relative_measurement(
            self.state[:3], self.state[landmark_slice]
        )
        cosine = math.cos(self.state[2])
        sine = math.sin(self.state[2])

        jacobian = np.zeros((2, self.dimension), dtype=float)
        jacobian[:, :3] = np.array(
            [
                [-cosine, -sine, prediction[1]],
                [sine, -cosine, -prediction[0]],
            ],
            dtype=float,
        )
        jacobian[:, landmark_slice] = np.array(
            [[cosine, sine], [-sine, cosine]], dtype=float
        )
        return prediction, jacobian

    def initialize_landmark(
        self,
        tag_id: int,
        measurement: Iterable[float],
        measurement_covariance: Iterable[Iterable[float]],
    ) -> None:
        """Augment the state and preserve cross-covariance with old states."""
        tag_id = int(tag_id)
        if tag_id in self._landmark_slices:
            raise ValueError(f"landmark {tag_id} already exists")
        observation = np.asarray(tuple(measurement), dtype=float)
        if observation.shape != (2,) or not np.all(np.isfinite(observation)):
            raise ValueError("measurement must contain two finite values")
        measurement_covariance_array = _as_covariance(
            measurement_covariance, 2
        )

        x, y, yaw = self.state[:3]
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        rotation = np.array([[cosine, -sine], [sine, cosine]], dtype=float)
        landmark_position = np.array([x, y], dtype=float) + rotation @ observation

        state_jacobian = np.array(
            [
                [1.0, 0.0, -sine * observation[0] - cosine * observation[1]],
                [0.0, 1.0, cosine * observation[0] - sine * observation[1]],
            ],
            dtype=float,
        )
        old_dimension = self.dimension
        landmark_to_old = state_jacobian @ self.covariance[:3, :]
        old_to_landmark = landmark_to_old.T
        landmark_covariance = (
            state_jacobian
            @ self.covariance[:3, :3]
            @ state_jacobian.T
            + rotation
            @ measurement_covariance_array
            @ rotation.T
        )

        augmented_covariance = np.zeros(
            (old_dimension + 2, old_dimension + 2), dtype=float
        )
        augmented_covariance[:old_dimension, :old_dimension] = self.covariance
        augmented_covariance[:old_dimension, old_dimension:] = old_to_landmark
        augmented_covariance[old_dimension:, :old_dimension] = landmark_to_old
        augmented_covariance[old_dimension:, old_dimension:] = landmark_covariance

        self.state = np.concatenate((self.state, landmark_position))
        self.covariance = augmented_covariance
        self._landmark_slices[tag_id] = slice(old_dimension, old_dimension + 2)
        self._stabilize_covariance()

    def observe(
        self,
        tag_id: int,
        measurement: Iterable[float],
        measurement_covariance: Iterable[Iterable[float]],
        gate_chi2: Optional[float] = 9.21034037197618,
    ) -> ObservationResult:
        """Initialize or update one tag observation."""
        tag_id = int(tag_id)
        observation = np.asarray(tuple(measurement), dtype=float)
        if observation.shape != (2,) or not np.all(np.isfinite(observation)):
            raise ValueError("measurement must contain two finite values")
        measurement_covariance_array = _as_covariance(
            measurement_covariance, 2
        )

        if tag_id not in self._landmark_slices:
            self.initialize_landmark(
                tag_id, observation, measurement_covariance_array
            )
            return ObservationResult(
                tag_id=tag_id,
                accepted=True,
                initialized=True,
                innovation=(0.0, 0.0),
                nis=None,
                reason="initialized",
            )

        prediction, measurement_jacobian = self.measurement_model(tag_id)
        innovation = observation - prediction
        innovation_covariance = (
            measurement_jacobian
            @ self.covariance
            @ measurement_jacobian.T
            + measurement_covariance_array
        )
        solved_innovation = np.linalg.solve(innovation_covariance, innovation)
        nis = float(innovation.T @ solved_innovation)

        if gate_chi2 is not None and nis > float(gate_chi2):
            return ObservationResult(
                tag_id=tag_id,
                accepted=False,
                initialized=False,
                innovation=(float(innovation[0]), float(innovation[1])),
                nis=nis,
                reason="innovation_gate",
            )

        gain = np.linalg.solve(
            innovation_covariance,
            measurement_jacobian @ self.covariance.T,
        ).T
        self.state = self.state + gain @ innovation
        self.state[2] = wrap_angle(self.state[2])

        identity = np.eye(self.dimension, dtype=float)
        correction = identity - gain @ measurement_jacobian
        with np.errstate(over="ignore", divide="ignore", invalid="ignore"):
            corrected_covariance = (
                correction @ self.covariance @ correction.T
                + gain @ measurement_covariance_array @ gain.T
            )
        if not np.all(np.isfinite(corrected_covariance)):
            raise FloatingPointError("non-finite covariance after update")
        self.covariance = corrected_covariance
        self._stabilize_covariance()

        return ObservationResult(
            tag_id=tag_id,
            accepted=True,
            initialized=False,
            innovation=(float(innovation[0]), float(innovation[1])),
            nis=nis,
            reason="updated",
        )

    def _stabilize_covariance(self) -> None:
        self.covariance = 0.5 * (self.covariance + self.covariance.T)

    def snapshot(self) -> dict:
        """Return a JSON-serializable state and marginal covariance snapshot."""
        landmarks = {}
        for tag_id in self.landmark_ids:
            position, covariance = self.landmark(tag_id)
            landmarks[str(tag_id)] = {
                "position": position.tolist(),
                "covariance": covariance.tolist(),
            }
        return {
            "robot": self.robot_pose.tolist(),
            "robot_covariance": self.robot_covariance.tolist(),
            "landmarks": landmarks,
            "state_dimension": self.dimension,
        }
