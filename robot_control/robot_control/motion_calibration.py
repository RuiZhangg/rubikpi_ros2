#!/usr/bin/env python3

"""Execute and estimate one supervised motion primitive from AprilTags."""

import argparse
from collections import defaultdict, deque
import csv
from dataclasses import dataclass
import json
import math
import os
import time

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from robot_control.landmark_motion import (
    bootstrap_uncertainty,
    estimate_motion,
    robust_centers,
)


@dataclass
class PendingDetection:
    stamp: Time
    family: str
    tag_id: int
    hamming: int
    decision_margin: float
    received_monotonic: float


class MotionCalibrationNode(Node):
    def __init__(self, arguments):
        super().__init__("motion_calibration_node")
        self.arguments = arguments
        self.command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            AprilTagDetectionArray,
            "/detections",
            self.on_detections,
            10,
        )

        self.pending = deque()
        self.before_samples = defaultdict(list)
        self.after_samples = defaultdict(list)
        self.statistics = defaultdict(int)
        self.last_tf_error = "none"
        self.last_status_time = 0.0
        self.phase = "collect_before"
        self.phase_start = time.monotonic()
        self.motion_start = None
        self.stop_time = None
        self.complete = False
        self.failure_reason = None
        self.result = None
        self.create_timer(0.05, self.control)

        if arguments.kind == "straight":
            self.linear_command = arguments.linear_speed
            self.angular_command = 0.0
            self.duration = arguments.distance / arguments.linear_speed
            self.commanded_distance = arguments.distance
            self.commanded_yaw = 0.0
        else:
            direction = 1.0 if arguments.kind == "left" else -1.0
            self.linear_command = 0.0
            self.angular_command = direction * arguments.turn_command
            self.duration = arguments.turn_pulse_s
            self.commanded_distance = 0.0
            self.commanded_yaw = direction * arguments.turn_increment_rad

        self.get_logger().warn(
            f"Trial {arguments.trial_id}: collecting stationary landmark data. "
            f"Motion is disabled until {arguments.minimum_common_tags} common "
            "tags have enough samples; keep Ctrl-C ready."
        )

    def on_detections(self, message):
        if self.phase not in ("collect_before", "collect_after"):
            return
        self.statistics["detection_arrays"] += 1
        self.statistics["detections"] += len(message.detections)
        stamp = Time.from_msg(message.header.stamp)
        received = time.monotonic()
        for detection in message.detections:
            self.pending.append(
                PendingDetection(
                    stamp=stamp,
                    family=str(detection.family),
                    tag_id=int(detection.id),
                    hamming=int(detection.hamming),
                    decision_margin=float(detection.decision_margin),
                    received_monotonic=received,
                )
            )

    def publish_command(self, linear, angular):
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self.command_publisher.publish(message)

    def process_pending(self):
        destination = (
            self.before_samples
            if self.phase == "collect_before"
            else self.after_samples
        )
        for _ in range(len(self.pending)):
            pending = self.pending.popleft()
            age = time.monotonic() - pending.received_monotonic
            if pending.hamming > self.arguments.max_hamming:
                self.statistics["hamming_rejections"] += 1
                continue
            if pending.decision_margin < self.arguments.min_decision_margin:
                self.statistics["margin_rejections"] += 1
                continue
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.arguments.base_frame,
                    f"{pending.family}:{pending.tag_id}",
                    pending.stamp,
                    timeout=Duration(seconds=self.arguments.tf_timeout_s),
                )
            except Exception as error:
                self.statistics["tf_failures"] += 1
                self.last_tf_error = str(error)
                if age < self.arguments.max_pending_age_s:
                    self.pending.append(pending)
                continue
            translation = transform.transform.translation
            self.statistics["tf_samples"] += 1
            destination[pending.tag_id].append(
                np.array([translation.x, translation.y], dtype=float)
            )

    def sufficiently_sampled_before(self):
        count = sum(
            len(samples) >= self.arguments.minimum_samples_per_tag
            for samples in self.before_samples.values()
        )
        if count < self.arguments.minimum_common_tags:
            return False
        centers = robust_centers(
            self.before_samples,
            self.arguments.minimum_samples_per_tag,
        )
        try:
            estimate_motion(
                centers,
                centers,
                minimum_baseline_m=self.arguments.minimum_baseline_m,
            )
        except ValueError:
            return False
        return True

    def sufficiently_sampled_after(self):
        count = sum(
            len(self.before_samples[tag_id])
            >= self.arguments.minimum_samples_per_tag
            and len(samples) >= self.arguments.minimum_samples_per_tag
            for tag_id, samples in self.after_samples.items()
        )
        return count >= self.arguments.minimum_common_tags

    def fail(self, reason):
        self.publish_command(0.0, 0.0)
        self.failure_reason = reason
        self.complete = True
        self.get_logger().error(reason)

    @staticmethod
    def sample_counts(samples_by_tag):
        return {
            int(tag_id): len(samples)
            for tag_id, samples in sorted(samples_by_tag.items())
        }

    def diagnostic_status(self):
        return (
            f"phase={self.phase}, detection_publishers="
            f"{self.count_publishers('/detections')}, "
            f"arrays={self.statistics['detection_arrays']}, "
            f"detections={self.statistics['detections']}, "
            f"tf_samples={self.statistics['tf_samples']}, "
            f"tf_failures={self.statistics['tf_failures']}, "
            f"hamming_rejections={self.statistics['hamming_rejections']}, "
            f"margin_rejections={self.statistics['margin_rejections']}, "
            f"before={self.sample_counts(self.before_samples)}, "
            f"after={self.sample_counts(self.after_samples)}, "
            f"last_tf_error={self.last_tf_error}"
        )

    def start_motion(self, now):
        self.phase = "moving"
        self.motion_start = now
        self.pending.clear()
        self.get_logger().warn("LANDMARK BASELINE READY; MOTION START")

    def finish_motion(self, now):
        self.publish_command(0.0, 0.0)
        self.phase = "settling"
        self.stop_time = now
        self.pending.clear()
        self.get_logger().warn("MOTION STOP; settling before final observations")

    def finish_estimate(self):
        before_centers = robust_centers(
            self.before_samples,
            self.arguments.minimum_samples_per_tag,
        )
        after_centers = robust_centers(
            self.after_samples,
            self.arguments.minimum_samples_per_tag,
        )
        try:
            estimate = estimate_motion(
                before_centers,
                after_centers,
                minimum_baseline_m=self.arguments.minimum_baseline_m,
            )
            uncertainty = bootstrap_uncertainty(
                self.before_samples,
                self.after_samples,
                self.arguments.minimum_samples_per_tag,
                self.arguments.minimum_baseline_m,
                resamples=self.arguments.bootstrap_resamples,
            )
        except ValueError as error:
            self.fail(str(error))
            return

        self.result = {
            "trial_id": self.arguments.trial_id,
            "kind": self.arguments.kind,
            "commanded_distance_m": self.commanded_distance,
            # The motion model's delta-s is signed displacement along the
            # starting forward axis. Total planar magnitude remains distance_m.
            "actual_distance_m": estimate.forward_m,
            "commanded_delta_yaw_rad": self.commanded_yaw,
            "actual_delta_yaw_rad": estimate.delta_yaw_rad,
            **estimate.as_dict(),
            **uncertainty,
            "before_sample_count": sum(
                len(self.before_samples[tag_id]) for tag_id in estimate.tag_ids
            ),
            "after_sample_count": sum(
                len(self.after_samples[tag_id]) for tag_id in estimate.tag_ids
            ),
            "notes": "estimated from common AprilTag centers",
        }
        self.append_result(self.result)
        print(json.dumps(self.result, indent=2, sort_keys=True), flush=True)
        self.get_logger().info(
            f"Estimated motion: forward={estimate.forward_m:.4f} m, "
            f"left={estimate.left_m:.4f} m, "
            f"yaw={math.degrees(estimate.delta_yaw_rad):.3f} deg, "
            f"fit RMSE={estimate.fit_rmse_m:.4f} m"
        )
        self.complete = True

    def append_result(self, result):
        path = os.path.expanduser(self.arguments.output_csv)
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        fields = [
            "trial_id",
            "kind",
            "commanded_distance_m",
            "actual_distance_m",
            "commanded_delta_yaw_rad",
            "actual_delta_yaw_rad",
            "forward_m",
            "left_m",
            "delta_yaw_deg",
            "fit_rmse_m",
            "fit_max_residual_m",
            "landmark_baseline_m",
            "forward_std_m",
            "left_std_m",
            "yaw_std_rad",
            "yaw_std_deg",
            "tag_ids",
            "before_sample_count",
            "after_sample_count",
            "notes",
        ]
        write_header = not os.path.exists(path) or os.path.getsize(path) == 0
        row = dict(result)
        row["tag_ids"] = ";".join(str(value) for value in result["tag_ids"])
        with open(path, "a", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
            if write_header:
                writer.writeheader()
            writer.writerow(row)
        self.get_logger().info(f"Appended trial to {path}")

    def control(self):
        now = time.monotonic()
        if self.complete:
            self.publish_command(0.0, 0.0)
            return

        if self.phase in ("collect_before", "collect_after"):
            self.publish_command(0.0, 0.0)
            self.process_pending()
            if now - self.last_status_time >= 2.0:
                self.last_status_time = now
                self.get_logger().info(self.diagnostic_status())
            elapsed = now - self.phase_start
            ready = (
                self.sufficiently_sampled_before()
                if self.phase == "collect_before"
                else self.sufficiently_sampled_after()
            )
            if ready and elapsed >= self.arguments.collection_s:
                if self.phase == "collect_before":
                    self.start_motion(now)
                else:
                    self.finish_estimate()
            elif elapsed >= self.arguments.max_observation_wait_s:
                self.fail(
                    f"Timed out in {self.phase}; need "
                    f"{self.arguments.minimum_common_tags} common tags with "
                    f"{self.arguments.minimum_samples_per_tag} samples each; "
                    f"diagnostics: {self.diagnostic_status()}"
                )
            return

        if self.phase == "moving":
            if now - self.motion_start < self.duration:
                self.publish_command(self.linear_command, self.angular_command)
            else:
                self.finish_motion(now)
            return

        self.publish_command(0.0, 0.0)
        if now - self.stop_time >= self.arguments.settle_s:
            self.phase = "collect_after"
            self.phase_start = now
            self.pending.clear()
            self.get_logger().info("Collecting final stationary landmarks")

    def destroy_node(self):
        self.publish_command(0.0, 0.0)
        super().destroy_node()


def parse_arguments(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("kind", choices=("straight", "left", "right"))
    parser.add_argument("--trial-id", default=time.strftime("trial_%Y%m%d_%H%M%S"))
    parser.add_argument("--distance", type=float, default=0.05)
    parser.add_argument("--linear-speed", type=float, default=0.04)
    parser.add_argument("--turn-command", type=float, default=0.002)
    parser.add_argument("--turn-pulse-s", type=float, default=0.10)
    parser.add_argument("--turn-increment-rad", type=float, default=math.radians(3.0))
    parser.add_argument("--settle-s", type=float, default=0.75)
    parser.add_argument("--collection-s", type=float, default=3.0)
    parser.add_argument("--minimum-samples-per-tag", type=int, default=10)
    parser.add_argument("--minimum-common-tags", type=int, default=2)
    parser.add_argument("--minimum-baseline-m", type=float, default=0.30)
    parser.add_argument("--max-observation-wait-s", type=float, default=60.0)
    parser.add_argument("--bootstrap-resamples", type=int, default=300)
    parser.add_argument("--max-hamming", type=int, default=1)
    parser.add_argument("--min-decision-margin", type=float, default=20.0)
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--tf-timeout-s", type=float, default=0.05)
    parser.add_argument("--max-pending-age-s", type=float, default=1.5)
    parser.add_argument(
        "--output-csv",
        default="~/hw3_logs/motion_calibration/landmark_motion_trials.csv",
    )
    parsed, ros_arguments = parser.parse_known_args(args)
    if parsed.distance <= 0.0 or parsed.linear_speed <= 0.0:
        parser.error("distance and linear speed must be positive")
    if parsed.turn_pulse_s <= 0.0 or parsed.turn_increment_rad <= 0.0:
        parser.error("turn pulse and increment must be positive")
    if parsed.minimum_samples_per_tag < 2 or parsed.minimum_common_tags < 2:
        parser.error("at least two samples and two common tags are required")
    return parsed, ros_arguments


def main(args=None):
    arguments, ros_arguments = parse_arguments(args)
    rclpy.init(args=ros_arguments)
    node = MotionCalibrationNode(arguments)
    try:
        while rclpy.ok() and not node.complete:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.failure_reason = "interrupted"
    finally:
        failed = node.failure_reason is not None
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
