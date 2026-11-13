#!/usr/bin/env python3

"""ROS 2 wrapper around the ROS-independent EKF-SLAM core."""

import csv
from collections import deque
from dataclasses import dataclass
import json
import math
import os
import time

import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String, UInt64
from tf2_ros import Buffer, TransformListener

from robot_control.ekf_slam import EKFSLAM


@dataclass
class PendingDetection:
    stamp: Time
    stamp_seconds: float
    family: str
    tag_id: int
    hamming: int
    decision_margin: float
    received_monotonic: float


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


class HW3SLAMNode(Node):
    def __init__(self):
        super().__init__("hw3_slam_node")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("max_hamming", 1)
        self.declare_parameter("min_decision_margin", 20.0)
        self.declare_parameter("tf_timeout_s", 0.05)
        self.declare_parameter("max_pending_age_s", 1.5)
        self.declare_parameter("gate_chi2", 9.21034037197618)
        self.declare_parameter("measurement_sigma_base_m", 0.025)
        self.declare_parameter("measurement_sigma_per_meter", 0.015)
        self.declare_parameter("process_sigma_distance_base_m", 0.015)
        self.declare_parameter("process_sigma_distance_per_meter", 0.15)
        self.declare_parameter("process_sigma_yaw_base_rad", 0.04)
        self.declare_parameter("process_sigma_yaw_per_rad", 0.25)
        self.declare_parameter("initial_pose_sigma_xy_m", 0.0001)
        self.declare_parameter("initial_pose_sigma_yaw_rad", 0.0001)
        self.declare_parameter("log_root", "~/hw3_logs")
        self.declare_parameter("run_name", "")

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.max_hamming = int(self.get_parameter("max_hamming").value)
        self.min_decision_margin = float(
            self.get_parameter("min_decision_margin").value
        )
        self.tf_timeout_s = float(self.get_parameter("tf_timeout_s").value)
        self.max_pending_age_s = float(
            self.get_parameter("max_pending_age_s").value
        )
        self.gate_chi2 = float(self.get_parameter("gate_chi2").value)
        self.measurement_sigma_base_m = float(
            self.get_parameter("measurement_sigma_base_m").value
        )
        self.measurement_sigma_per_meter = float(
            self.get_parameter("measurement_sigma_per_meter").value
        )
        self.process_sigma_distance_base_m = float(
            self.get_parameter("process_sigma_distance_base_m").value
        )
        self.process_sigma_distance_per_meter = float(
            self.get_parameter("process_sigma_distance_per_meter").value
        )
        self.process_sigma_yaw_base_rad = float(
            self.get_parameter("process_sigma_yaw_base_rad").value
        )
        self.process_sigma_yaw_per_rad = float(
            self.get_parameter("process_sigma_yaw_per_rad").value
        )

        initial_xy_sigma = float(
            self.get_parameter("initial_pose_sigma_xy_m").value
        )
        initial_yaw_sigma = float(
            self.get_parameter("initial_pose_sigma_yaw_rad").value
        )
        self.filter = EKFSLAM(
            initial_pose_covariance=np.diag(
                [
                    initial_xy_sigma**2,
                    initial_xy_sigma**2,
                    initial_yaw_sigma**2,
                ]
            )
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pending = deque()
        self.update_count = 0

        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/hw3/slam_pose", 10
        )
        self.map_publisher = self.create_publisher(String, "/hw3/slam_map", 10)
        self.update_publisher = self.create_publisher(
            UInt64, "/hw3/slam_update_count", 10
        )
        self.create_subscription(
            AprilTagDetectionArray,
            "/detections",
            self.on_detections,
            10,
        )
        self.create_subscription(
            Vector3,
            "/hw3/control_increment",
            self.on_control_increment,
            10,
        )
        self.create_timer(0.05, self.process_pending)

        self.run_directory = self._make_run_directory()
        self.state_file = open(
            os.path.join(self.run_directory, "slam_states.jsonl"),
            "w",
            encoding="utf-8",
        )
        self.observation_file = open(
            os.path.join(self.run_directory, "observations.csv"),
            "w",
            newline="",
            encoding="utf-8",
        )
        self.observation_writer = csv.writer(self.observation_file)
        self.observation_writer.writerow(
            [
                "stamp_s",
                "tag_id",
                "range_m",
                "z_forward_m",
                "z_left_m",
                "sigma_m",
                "decision_margin",
                "hamming",
                "accepted",
                "initialized",
                "nis",
                "reason",
            ]
        )

        self.publish_snapshot("startup")
        self.get_logger().info(
            "HW3 EKF-SLAM started with an empty landmark map. "
            f"Logs: {self.run_directory}"
        )

    def _make_run_directory(self):
        root = os.path.expanduser(str(self.get_parameter("log_root").value))
        run_name = str(self.get_parameter("run_name").value).strip()
        if not run_name:
            run_name = time.strftime("run_%Y%m%d_%H%M%S")
        path = os.path.join(root, run_name)
        os.makedirs(path, exist_ok=True)
        return path

    def on_detections(self, message):
        stamp = Time.from_msg(message.header.stamp)
        stamp_seconds = stamp.nanoseconds / 1e9
        received = time.monotonic()
        for detection in message.detections:
            self.pending.append(
                PendingDetection(
                    stamp=stamp,
                    stamp_seconds=stamp_seconds,
                    family=str(detection.family),
                    tag_id=int(detection.id),
                    hamming=int(detection.hamming),
                    decision_margin=float(detection.decision_margin),
                    received_monotonic=received,
                )
            )

    def on_control_increment(self, message):
        distance = float(message.x)
        delta_yaw = float(message.y)
        distance_sigma = (
            self.process_sigma_distance_base_m
            + self.process_sigma_distance_per_meter * abs(distance)
        )
        yaw_sigma = (
            self.process_sigma_yaw_base_rad
            + self.process_sigma_yaw_per_rad * abs(delta_yaw)
        )
        control_covariance = np.diag([distance_sigma**2, yaw_sigma**2])
        self.filter.predict(distance, delta_yaw, control_covariance)
        self.publish_snapshot("prediction")

    def process_pending(self):
        for _ in range(len(self.pending)):
            pending = self.pending.popleft()
            age = time.monotonic() - pending.received_monotonic

            if pending.hamming > self.max_hamming:
                self._log_rejected_detection(pending, "hamming_gate")
                continue
            if pending.decision_margin < self.min_decision_margin:
                self._log_rejected_detection(pending, "decision_margin_gate")
                continue

            child_frame = f"{pending.family}:{pending.tag_id}"
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    child_frame,
                    pending.stamp,
                    timeout=Duration(seconds=self.tf_timeout_s),
                )
            except Exception:
                if age < self.max_pending_age_s:
                    self.pending.append(pending)
                else:
                    self._log_rejected_detection(pending, "missing_transform")
                continue

            translation = transform.transform.translation
            observation = np.array([translation.x, translation.y], dtype=float)
            range_m = math.sqrt(
                translation.x**2 + translation.y**2 + translation.z**2
            )
            sigma = (
                self.measurement_sigma_base_m
                + self.measurement_sigma_per_meter * range_m
            )
            measurement_covariance = np.diag([sigma**2, sigma**2])
            result = self.filter.observe(
                pending.tag_id,
                observation,
                measurement_covariance,
                gate_chi2=self.gate_chi2,
            )
            self.observation_writer.writerow(
                [
                    pending.stamp_seconds,
                    pending.tag_id,
                    range_m,
                    observation[0],
                    observation[1],
                    sigma,
                    pending.decision_margin,
                    pending.hamming,
                    int(result.accepted),
                    int(result.initialized),
                    "" if result.nis is None else result.nis,
                    result.reason,
                ]
            )
            self.observation_file.flush()

            if result.accepted:
                self.update_count += 1
                self.publish_snapshot(
                    "landmark_initialized" if result.initialized else "measurement_update"
                )
            else:
                self.get_logger().warn(
                    f"Rejected tag {pending.tag_id}: {result.reason}, "
                    f"NIS={result.nis:.2f}"
                )

    def _log_rejected_detection(self, pending, reason):
        self.observation_writer.writerow(
            [
                pending.stamp_seconds,
                pending.tag_id,
                "",
                "",
                "",
                "",
                pending.decision_margin,
                pending.hamming,
                0,
                0,
                "",
                reason,
            ]
        )
        self.observation_file.flush()

    def publish_snapshot(self, trigger):
        now = self.get_clock().now()
        snapshot = self.filter.snapshot()
        snapshot["stamp_s"] = now.nanoseconds / 1e9
        snapshot["trigger"] = trigger
        snapshot["update_count"] = self.update_count

        self.state_file.write(json.dumps(snapshot, sort_keys=True) + "\n")
        self.state_file.flush()

        pose_message = PoseWithCovarianceStamped()
        pose_message.header.stamp = now.to_msg()
        pose_message.header.frame_id = "slam"
        pose = self.filter.robot_pose
        pose_message.pose.pose.position.x = float(pose[0])
        pose_message.pose.pose.position.y = float(pose[1])
        qx, qy, qz, qw = yaw_to_quaternion(float(pose[2]))
        pose_message.pose.pose.orientation.x = qx
        pose_message.pose.pose.orientation.y = qy
        pose_message.pose.pose.orientation.z = qz
        pose_message.pose.pose.orientation.w = qw

        robot_covariance = self.filter.robot_covariance
        output_covariance = [0.0] * 36
        state_indices = [0, 1, 5]
        for row, output_row in enumerate(state_indices):
            for column, output_column in enumerate(state_indices):
                output_covariance[6 * output_row + output_column] = float(
                    robot_covariance[row, column]
                )
        pose_message.pose.covariance = output_covariance
        self.pose_publisher.publish(pose_message)

        map_message = String()
        map_message.data = json.dumps(snapshot, sort_keys=True)
        self.map_publisher.publish(map_message)

        count_message = UInt64()
        count_message.data = self.update_count
        self.update_publisher.publish(count_message)

    def destroy_node(self):
        try:
            self.state_file.close()
            self.observation_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HW3SLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
