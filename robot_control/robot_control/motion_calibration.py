#!/usr/bin/env python3

"""Execute exactly one supervised HW3 motion primitive for calibration."""

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MotionCalibrationNode(Node):
    def __init__(self, arguments):
        super().__init__("motion_calibration_node")
        self.arguments = arguments
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.start_time = time.monotonic()
        self.motion_start = None
        self.stop_time = None
        self.complete = False
        self.started = False
        self.create_timer(0.05, self.control)

        if arguments.kind == "straight":
            self.linear_command = arguments.linear_speed
            self.angular_command = 0.0
            self.duration = arguments.distance / arguments.linear_speed
            description = (
                f"straight {arguments.distance:.3f} m at "
                f"{arguments.linear_speed:.3f} m/s"
            )
        else:
            direction = 1.0 if arguments.kind == "left" else -1.0
            self.linear_command = 0.0
            self.angular_command = direction * arguments.turn_command
            self.duration = arguments.turn_pulse_s
            description = (
                f"{arguments.kind} pulse, command={self.angular_command:.6f}, "
                f"duration={self.duration:.3f} s"
            )

        self.get_logger().warn(
            f"Trial {arguments.trial_id}: one {description}. "
            f"Motion starts in {arguments.start_delay_s:.1f} s; keep Ctrl-C ready."
        )

    def publish_command(self, linear, angular):
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self.publisher.publish(message)

    def control(self):
        now = time.monotonic()
        if self.complete:
            self.publish_command(0.0, 0.0)
            return

        if not self.started:
            if now - self.start_time < self.arguments.start_delay_s:
                self.publish_command(0.0, 0.0)
                return
            self.started = True
            self.motion_start = now
            self.get_logger().warn("MOTION START")

        if now - self.motion_start < self.duration:
            self.publish_command(self.linear_command, self.angular_command)
            return

        self.publish_command(0.0, 0.0)
        if self.stop_time is None:
            self.stop_time = now
            self.get_logger().warn("MOTION STOP; settling with zero commands")
            return
        if now - self.stop_time >= self.arguments.settle_s:
            self.complete = True
            self.get_logger().info("Calibration primitive complete.")

    def destroy_node(self):
        self.publish_command(0.0, 0.0)
        super().destroy_node()


def parse_arguments(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("kind", choices=("straight", "left", "right"))
    parser.add_argument("--trial-id", default="unspecified")
    parser.add_argument("--distance", type=float, default=0.05)
    parser.add_argument("--linear-speed", type=float, default=0.04)
    parser.add_argument("--turn-command", type=float, default=0.002)
    parser.add_argument("--turn-pulse-s", type=float, default=0.10)
    parser.add_argument("--start-delay-s", type=float, default=3.0)
    parser.add_argument("--settle-s", type=float, default=0.50)
    parsed, ros_arguments = parser.parse_known_args(args)
    if parsed.distance <= 0.0 or parsed.linear_speed <= 0.0:
        parser.error("distance and linear speed must be positive")
    if parsed.turn_pulse_s <= 0.0:
        parser.error("turn pulse duration must be positive")
    if parsed.start_delay_s < 0.0 or parsed.settle_s < 0.0:
        parser.error("delay and settle time must be nonnegative")
    return parsed, ros_arguments


def main(args=None):
    arguments, ros_arguments = parse_arguments(args)
    rclpy.init(args=ros_arguments)
    node = MotionCalibrationNode(arguments)
    try:
        while rclpy.ok() and not node.complete:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
