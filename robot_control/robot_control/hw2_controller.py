#!/usr/bin/env python3

import argparse
import csv
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Int32


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class HW2Controller(Node):
    def __init__(self, stop_after):
        super().__init__("hw2_controller")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(
            PoseStamped,
            "/hw2/vision_pose",
            self.vision_callback,
            10,
        )

        self.create_subscription(
            Int32,
            "/hw2/vision_tag_id",
            self.tag_callback,
            10,
        )

        # Physical route: 0.8 m per course coordinate unit.
        self.waypoints = np.array([
            [0.0, 0.0, 0.0],
            [0.8, 0.0, 0.0],
            [0.8, 1.6, math.pi],
            [0.0, 0.0, 0.0],
        ], dtype=float)

        self.current_state = np.array([0.0, 0.0, 0.0], dtype=float)

        self.have_vision = False
        self.last_vision_time = None
        self.last_tag_id = -1

        # Blend visual correction into dead reckoning to avoid jumps.
        self.vision_alpha_xy = 0.50
        self.vision_alpha_yaw = 0.60

        # Reject clearly impossible single-frame jumps after initialization.
        self.max_vision_position_jump = 0.45   # m
        self.max_vision_yaw_jump = math.radians(45.0)

        # Controller settings.
        self.position_tolerance = 0.07          # m
        self.angle_tolerance = math.radians(6.0)

        self.max_linear = 0.14                  # m/s
        self.max_angular_drive = 0.70           # rad/s
        self.rotate_speed = 0.60                # rad/s

        self.kp_distance = 0.55
        self.kp_heading_drive = 1.30

        self.dt = 0.10
        self.waypoint_idx = 0
        self.stop_after = stop_after
        self.stage = "waiting_for_vision"

        self.last_linear_cmd = 0.0
        self.last_angular_cmd = 0.0

        self.start_monotonic = time.monotonic()

        log_dir = os.path.expanduser("~/hw2_logs")
        os.makedirs(log_dir, exist_ok=True)

        stamp = time.strftime("%Y%m%d_%H%M%S")

        self.trajectory_path = os.path.join(
            log_dir, f"trajectory_{stamp}.csv"
        )
        self.waypoint_path = os.path.join(
            log_dir, f"waypoints_{stamp}.csv"
        )

        self.trajectory_file = open(
            self.trajectory_path, "w", newline=""
        )
        self.trajectory_writer = csv.writer(self.trajectory_file)
        self.trajectory_writer.writerow([
            "time_s",
            "x_m",
            "y_m",
            "yaw_rad",
            "target_index",
            "target_x_m",
            "target_y_m",
            "target_yaw_rad",
            "stage",
            "linear_cmd_mps",
            "angular_cmd_radps",
            "vision_age_s",
            "tag_id",
        ])

        self.waypoint_file = open(
            self.waypoint_path, "w", newline=""
        )
        self.waypoint_writer = csv.writer(self.waypoint_file)
        self.waypoint_writer.writerow([
            "waypoint_index",
            "target_x_m",
            "target_y_m",
            "target_yaw_rad",
            "estimated_x_m",
            "estimated_y_m",
            "estimated_yaw_rad",
            "translation_error_m",
            "rotation_error_rad",
            "time_s",
        ])

        self.control_timer = self.create_timer(
            self.dt, self.control_loop
        )
        self.log_timer = self.create_timer(
            0.50, self.log_state
        )

        self.get_logger().info(
            "HW2 controller started; waiting for first /hw2/vision_pose."
        )
        self.get_logger().info(
            f"Trajectory log: {self.trajectory_path}"
        )
        self.get_logger().info(
            f"Waypoint log: {self.waypoint_path}"
        )

    def tag_callback(self, msg):
        self.last_tag_id = int(msg.data)

    def vision_callback(self, msg):
        measurement = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            quaternion_to_yaw(msg.pose.orientation),
        ], dtype=float)

        now = self.get_clock().now()

        if not self.have_vision:
            self.current_state = measurement.copy()
            self.have_vision = True
            self.last_vision_time = now

            self.get_logger().info(
                "Initialized from vision: "
                f"x={measurement[0]:.3f}, "
                f"y={measurement[1]:.3f}, "
                f"yaw={math.degrees(measurement[2]):.1f} deg"
            )

            # The experiment physically starts at waypoint 0. Record the
            # measured starting error but do not waste time "correcting"
            # back to the origin before beginning the route.
            self.record_waypoint(0, self.waypoints[0])

            if self.stop_after <= 0:
                self.waypoint_idx = len(self.waypoints)
                self.stop_robot()
                return

            self.waypoint_idx = 1
            self.stage = "rotate_to_goal"
            return

        pos_jump = math.hypot(
            measurement[0] - self.current_state[0],
            measurement[1] - self.current_state[1],
        )
        yaw_jump = abs(
            wrap_angle(measurement[2] - self.current_state[2])
        )

        if (
            pos_jump > self.max_vision_position_jump
            or yaw_jump > self.max_vision_yaw_jump
        ):
            self.get_logger().warn(
                "Rejected visual jump: "
                f"position={pos_jump:.3f} m, "
                f"yaw={math.degrees(yaw_jump):.1f} deg"
            )
            return

        a = self.vision_alpha_xy
        self.current_state[0] = (
            (1.0 - a) * self.current_state[0]
            + a * measurement[0]
        )
        self.current_state[1] = (
            (1.0 - a) * self.current_state[1]
            + a * measurement[1]
        )

        yaw_error = wrap_angle(
            measurement[2] - self.current_state[2]
        )
        self.current_state[2] = wrap_angle(
            self.current_state[2]
            + self.vision_alpha_yaw * yaw_error
        )

        self.last_vision_time = now

    def desired_heading(self, target):
        dx = target[0] - self.current_state[0]
        dy = target[1] - self.current_state[1]
        return math.atan2(dy, dx)

    def update_dead_reckoning(self, linear, angular):
        theta = self.current_state[2]

        self.current_state[0] += (
            linear * math.cos(theta) * self.dt
        )
        self.current_state[1] += (
            linear * math.sin(theta) * self.dt
        )
        self.current_state[2] = wrap_angle(
            self.current_state[2] + angular * self.dt
        )

    def stop_robot(self):
        msg = Twist()
        self.cmd_pub.publish(msg)
        self.last_linear_cmd = 0.0
        self.last_angular_cmd = 0.0

    def record_waypoint(self, idx, target):
        dx = target[0] - self.current_state[0]
        dy = target[1] - self.current_state[1]
        trans_error = math.hypot(dx, dy)

        rot_error = abs(
            wrap_angle(target[2] - self.current_state[2])
        )

        elapsed = time.monotonic() - self.start_monotonic

        self.waypoint_writer.writerow([
            idx,
            target[0],
            target[1],
            target[2],
            self.current_state[0],
            self.current_state[1],
            self.current_state[2],
            trans_error,
            rot_error,
            elapsed,
        ])
        self.waypoint_file.flush()

        self.get_logger().info(
            f"WAYPOINT {idx} reached: "
            f"pose=({self.current_state[0]:.3f}, "
            f"{self.current_state[1]:.3f}, "
            f"{math.degrees(self.current_state[2]):.1f} deg), "
            f"position_error={trans_error:.3f} m, "
            f"rotation_error={math.degrees(rot_error):.1f} deg"
        )

    def control_loop(self):
        if not self.have_vision:
            self.stop_robot()
            return

        if self.waypoint_idx >= len(self.waypoints):
            self.stop_robot()
            return

        target = self.waypoints[self.waypoint_idx]

        dx = target[0] - self.current_state[0]
        dy = target[1] - self.current_state[1]
        distance = math.hypot(dx, dy)

        msg = Twist()

        # Once position is close enough, only correct final orientation.
        if distance <= self.position_tolerance:
            self.stage = "rotate_to_orientation"

            yaw_error = wrap_angle(
                target[2] - self.current_state[2]
            )

            if abs(yaw_error) <= self.angle_tolerance:
                self.stop_robot()
                reached_idx = self.waypoint_idx
                self.record_waypoint(
                    reached_idx, target
                )

                if reached_idx >= self.stop_after:
                    self.get_logger().info(
                        f"Stopping after waypoint {reached_idx} as requested."
                    )
                    self.waypoint_idx = len(self.waypoints)
                    self.stop_robot()
                    return

                self.waypoint_idx += 1

                if self.waypoint_idx >= len(self.waypoints):
                    self.get_logger().info(
                        "All HW2 waypoints reached."
                    )
                    return

                self.stage = "rotate_to_goal"
                return

            msg.angular.z = math.copysign(
                self.rotate_speed, yaw_error
            )

        else:
            desired = self.desired_heading(target)
            heading_error = wrap_angle(
                desired - self.current_state[2]
            )

            # If badly misaligned, rotate in place first.
            if abs(heading_error) > math.radians(20.0):
                self.stage = "rotate_to_goal"
                msg.angular.z = math.copysign(
                    self.rotate_speed, heading_error
                )

            else:
                self.stage = "drive"

                linear = self.kp_distance * distance
                linear = min(linear, self.max_linear)

                # Slow down as heading error grows.
                linear *= max(
                    0.35,
                    math.cos(abs(heading_error))
                )

                angular = (
                    self.kp_heading_drive * heading_error
                )
                angular = max(
                    -self.max_angular_drive,
                    min(self.max_angular_drive, angular),
                )

                msg.linear.x = float(linear)
                msg.angular.z = float(angular)

        self.last_linear_cmd = msg.linear.x
        self.last_angular_cmd = msg.angular.z

        # Predict first; the next vision callback corrects this prediction.
        self.update_dead_reckoning(
            self.last_linear_cmd,
            self.last_angular_cmd,
        )

        self.cmd_pub.publish(msg)

    def log_state(self):
        if not self.have_vision:
            return

        elapsed = time.monotonic() - self.start_monotonic

        if self.waypoint_idx < len(self.waypoints):
            target = self.waypoints[self.waypoint_idx]
            target_idx = self.waypoint_idx
        else:
            target = self.waypoints[-1]
            target_idx = len(self.waypoints)

        if self.last_vision_time is None:
            vision_age = float("inf")
        else:
            vision_age = (
                self.get_clock().now()
                - self.last_vision_time
            ).nanoseconds / 1e9

        self.trajectory_writer.writerow([
            elapsed,
            self.current_state[0],
            self.current_state[1],
            self.current_state[2],
            target_idx,
            target[0],
            target[1],
            target[2],
            self.stage,
            self.last_linear_cmd,
            self.last_angular_cmd,
            vision_age,
            self.last_tag_id,
        ])
        self.trajectory_file.flush()

    def destroy_node(self):
        try:
            self.stop_robot()
        except Exception:
            pass

        try:
            self.trajectory_file.close()
            self.waypoint_file.close()
        except Exception:
            pass

        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--stop-after",
        type=int,
        default=3,
        choices=[0, 1, 2, 3],
        help="stop after reaching this waypoint index; 3 runs the full route",
    )
    args = parser.parse_args()

    rclpy.init()
    node = HW2Controller(args.stop_after)

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
