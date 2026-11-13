#!/usr/bin/env python3

"""Stop-observe-update closed-loop square/octagon controller for HW3."""

import csv
import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Vector3
from rclpy.node import Node
from std_msgs.msg import String, UInt64

from robot_control.ekf_slam import wrap_angle
from robot_control.trajectory import build_waypoints


def quaternion_to_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
    )


class HW3Controller(Node):
    def __init__(self):
        super().__init__("hw3_controller")

        self.declare_parameter("shape", "square")
        self.declare_parameter("side_length_m", 1.8)
        self.declare_parameter("laps", 1)
        self.declare_parameter("step_distance_m", 0.05)
        self.declare_parameter("drive_speed_mps", 0.04)
        self.declare_parameter("translation_distance_scale", 1.0)
        self.declare_parameter("turn_command", 0.002)
        self.declare_parameter("turn_pulse_s", 0.10)
        self.declare_parameter("left_turn_increment_rad", math.radians(3.0))
        self.declare_parameter("right_turn_increment_rad", math.radians(3.0))
        self.declare_parameter("turn_tolerance_rad", math.radians(2.0))
        self.declare_parameter("waypoint_tolerance_m", 0.07)
        self.declare_parameter("observation_timeout_s", 6.0)
        self.declare_parameter("max_blind_turn_pulses", 30)
        self.declare_parameter("start_delay_s", 3.0)
        self.declare_parameter("minimum_landmarks_to_start", 1)
        self.declare_parameter("log_root", "~/hw3_logs")
        self.declare_parameter("run_name", "")

        self.shape = str(self.get_parameter("shape").value).lower()
        self.side_length_m = float(self.get_parameter("side_length_m").value)
        self.laps = int(self.get_parameter("laps").value)
        self.step_distance_m = float(
            self.get_parameter("step_distance_m").value
        )
        self.drive_speed_mps = float(self.get_parameter("drive_speed_mps").value)
        self.translation_distance_scale = float(
            self.get_parameter("translation_distance_scale").value
        )
        self.turn_command = float(self.get_parameter("turn_command").value)
        self.turn_pulse_s = float(self.get_parameter("turn_pulse_s").value)
        self.left_turn_increment_rad = float(
            self.get_parameter("left_turn_increment_rad").value
        )
        self.right_turn_increment_rad = float(
            self.get_parameter("right_turn_increment_rad").value
        )
        self.turn_tolerance_rad = float(
            self.get_parameter("turn_tolerance_rad").value
        )
        self.waypoint_tolerance_m = float(
            self.get_parameter("waypoint_tolerance_m").value
        )
        self.observation_timeout_s = float(
            self.get_parameter("observation_timeout_s").value
        )
        self.max_blind_turn_pulses = int(
            self.get_parameter("max_blind_turn_pulses").value
        )
        self.start_delay_s = float(self.get_parameter("start_delay_s").value)
        self.minimum_landmarks_to_start = int(
            self.get_parameter("minimum_landmarks_to_start").value
        )

        self.waypoints = build_waypoints(
            self.shape, self.side_length_m, self.laps
        )
        self.target_index = 1
        self.pose = None
        self.update_count = 0
        self.landmark_count = 0
        self.mode = "waiting_for_map"
        self.start_monotonic = time.monotonic()
        self.motion_deadline = 0.0
        self.wait_deadline = 0.0
        self.baseline_update_count = 0
        self.active_linear = 0.0
        self.active_angular = 0.0
        self.pending_distance = 0.0
        self.pending_delta_yaw = 0.0
        self.turn_target_yaw = None
        self.blind_turn_pulses = 0
        self.last_status = 0.0

        self.command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.increment_publisher = self.create_publisher(
            Vector3, "/hw3/control_increment", 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/hw3/slam_pose",
            self.on_pose,
            10,
        )
        self.create_subscription(
            UInt64,
            "/hw3/slam_update_count",
            self.on_update_count,
            10,
        )
        self.create_subscription(String, "/hw3/slam_map", self.on_map, 10)
        self.create_timer(0.05, self.control)

        self.event_file = self._open_event_log()
        self.event_writer = csv.writer(self.event_file)
        self.event_writer.writerow(
            [
                "monotonic_s",
                "event",
                "mode",
                "target_index",
                "x_m",
                "y_m",
                "yaw_rad",
                "linear_cmd",
                "angular_cmd",
                "predicted_distance_m",
                "predicted_delta_yaw_rad",
                "update_count",
            ]
        )
        self._record_event("startup")
        self.get_logger().info(
            f"HW3 controller ready: shape={self.shape}, "
            f"side={self.side_length_m:.3f} m, laps={self.laps}."
        )

    def _open_event_log(self):
        root = os.path.expanduser(str(self.get_parameter("log_root").value))
        run_name = str(self.get_parameter("run_name").value).strip()
        if not run_name:
            run_name = time.strftime("controller_%Y%m%d_%H%M%S")
        directory = os.path.join(root, run_name)
        os.makedirs(directory, exist_ok=True)
        return open(
            os.path.join(directory, "controller_events.csv"),
            "w",
            newline="",
            encoding="utf-8",
        )

    def on_pose(self, message):
        self.pose = np.array(
            [
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                quaternion_to_yaw(message.pose.pose.orientation),
            ],
            dtype=float,
        )

    def on_update_count(self, message):
        self.update_count = int(message.data)

    def on_map(self, message):
        try:
            snapshot = json.loads(message.data)
            self.landmark_count = len(snapshot.get("landmarks", {}))
        except (TypeError, ValueError):
            self.get_logger().warn("Ignored malformed /hw3/slam_map message")

    def publish_command(self, linear, angular):
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self.command_publisher.publish(message)

    def stop_robot(self):
        self.publish_command(0.0, 0.0)

    def _start_motion(self, linear, angular, duration, distance, delta_yaw):
        self.active_linear = float(linear)
        self.active_angular = float(angular)
        self.pending_distance = float(distance)
        self.pending_delta_yaw = float(delta_yaw)
        self.motion_deadline = time.monotonic() + float(duration)
        self.mode = "moving"
        self._record_event("motion_start")
        self.publish_command(self.active_linear, self.active_angular)

    def _finish_motion(self):
        self.stop_robot()
        increment = Vector3()
        increment.x = self.pending_distance
        increment.y = self.pending_delta_yaw
        increment.z = 0.0
        self.increment_publisher.publish(increment)
        self.baseline_update_count = self.update_count
        self.wait_deadline = time.monotonic() + self.observation_timeout_s
        self.mode = "waiting_for_observation"
        self._record_event("motion_complete")

    def control(self):
        now = time.monotonic()

        if self.mode == "complete":
            self.stop_robot()
            return

        if self.mode == "moving":
            if now < self.motion_deadline:
                self.publish_command(self.active_linear, self.active_angular)
            else:
                self._finish_motion()
            return

        if self.mode == "waiting_for_observation":
            self.stop_robot()
            if self.update_count > self.baseline_update_count:
                self.mode = "decide"
                self.blind_turn_pulses = 0
                self._record_event("fresh_observation")
                if (
                    abs(self.pending_delta_yaw) > 0.0
                    and self.turn_target_yaw is not None
                ):
                    remaining = wrap_angle(self.turn_target_yaw - self.pose[2])
                    self.get_logger().info(
                        "Landmark update accepted during turn: "
                        f"yaw={math.degrees(self.pose[2]):.1f} deg, "
                        f"target={math.degrees(self.turn_target_yaw):.1f} deg, "
                        f"remaining={math.degrees(remaining):.1f} deg."
                    )
            elif now > self.wait_deadline:
                if (
                    abs(self.pending_delta_yaw) > 0.0
                    and self.turn_target_yaw is not None
                    and self.blind_turn_pulses < self.max_blind_turn_pulses
                ):
                    heading_error = wrap_angle(
                        self.turn_target_yaw - self.pose[2]
                    )
                    if abs(heading_error) > self.turn_tolerance_rad:
                        self.blind_turn_pulses += 1
                        self._record_event("blind_turn_search")
                        self._start_turn(heading_error, preserve_target=True)
                        return
                self._status(
                    "No accepted landmark update after motion; holding still. "
                    "Blind translation is disabled."
                )
            return

        if (
            self.pose is None
            or self.update_count < 1
            or self.landmark_count < self.minimum_landmarks_to_start
            or now - self.start_monotonic < self.start_delay_s
        ):
            self.stop_robot()
            self._status(
                "Waiting for accepted observations from at least "
                f"{self.minimum_landmarks_to_start} mapped landmarks; "
                f"currently mapped={self.landmark_count}."
            )
            return

        if self.mode == "waiting_for_map":
            self.mode = "decide"
            self._record_event("controller_started")

        if self.mode == "final_alignment":
            heading_error = wrap_angle(-self.pose[2])
            if abs(heading_error) <= self.turn_tolerance_rad:
                self.mode = "complete"
                self.stop_robot()
                self._record_event("trajectory_complete")
                self.get_logger().info(
                    "Trajectory complete and final yaw aligned with start."
                )
                return
            self._start_turn(heading_error)
            return

        if self.mode != "decide":
            self.stop_robot()
            return

        target = self.waypoints[self.target_index]
        delta = target - self.pose[:2]
        distance = float(np.linalg.norm(delta))

        if distance <= self.waypoint_tolerance_m:
            self._record_event("waypoint_reached")
            self.target_index += 1
            if self.target_index >= len(self.waypoints):
                self.mode = "final_alignment"
            return

        desired_heading = math.atan2(delta[1], delta[0])
        heading_error = wrap_angle(desired_heading - self.pose[2])
        if abs(heading_error) > self.turn_tolerance_rad:
            self._start_turn(heading_error)
            return

        step = min(self.step_distance_m, distance)
        duration = step / max(self.drive_speed_mps, 1e-6)
        self._start_motion(
            self.drive_speed_mps,
            0.0,
            duration,
            step * self.translation_distance_scale,
            0.0,
        )

    def _start_turn(self, heading_error, preserve_target=False):
        direction = 1.0 if heading_error > 0.0 else -1.0
        predicted_increment = (
            self.left_turn_increment_rad
            if direction > 0.0
            else self.right_turn_increment_rad
        )
        if not preserve_target:
            self.turn_target_yaw = wrap_angle(self.pose[2] + heading_error)
        self._start_motion(
            0.0,
            direction * self.turn_command,
            self.turn_pulse_s,
            0.0,
            direction * predicted_increment,
        )

    def _status(self, text):
        now = time.monotonic()
        if now - self.last_status >= 1.0:
            self.last_status = now
            self.get_logger().info(text)

    def _record_event(self, event):
        if not hasattr(self, "event_writer"):
            return
        if self.pose is None:
            pose = ["", "", ""]
        else:
            pose = self.pose.tolist()
        self.event_writer.writerow(
            [
                time.monotonic(),
                event,
                self.mode,
                self.target_index,
                pose[0],
                pose[1],
                pose[2],
                self.active_linear,
                self.active_angular,
                self.pending_distance,
                self.pending_delta_yaw,
                self.update_count,
            ]
        )
        self.event_file.flush()

    def destroy_node(self):
        try:
            self.stop_robot()
            self.event_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HW3Controller()
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
