#!/usr/bin/env python3

import argparse
import csv
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Int32


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def q_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def circ_mean(values):
    if not values:
        return None
    return math.atan2(
        sum(math.sin(v) for v in values),
        sum(math.cos(v) for v in values),
    )


class HW2Controller(Node):
    """
    Accuracy-first controller for very slow/intermittent vision and
    highly nonlinear turning.

    Core policy:
      * Translate with angular.z = 0.
      * Turn only while stopped.
      * One turn pulse = angular.z +/-0.002 for 0.10 s.
      * Calibrated pulse size ~= 3 deg.
      * Wait 1.8 s after each pulse for vision before deciding again.
      * Use known straight-line route geometry and cross-track error.
    """

    def __init__(self, stop_after):
        super().__init__("hw2_controller")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(
            PoseStamped,
            "/hw2/vision_pose",
            self.on_vision,
            10,
        )

        self.create_subscription(
            Int32,
            "/hw2/vision_tag_id",
            self.on_tag,
            10,
        )

        # ------------------------------------------------------
        # Route geometry
        # ------------------------------------------------------
        self.points = np.array([
            [0.0, 0.0],
            [0.8, 0.0],
            [0.8, 1.6],
            [0.0, 0.0],
        ], dtype=float)

        self.final_yaw = {
            0: 0.0,
            1: 0.0,
            2: math.pi,
            3: 0.0,
        }

        self.segment_heading = {
            1: 0.0,
            2: math.pi / 2.0,
            3: math.atan2(-1.6, -0.8),
        }

        self.segment_tag = {
            1: 1,
            2: 0,
            3: 2,
        }

        self.stop_after = stop_after
        self.wp = 0

        # x, y, yaw
        self.state = np.zeros(3, dtype=float)

        self.have_vision = False
        self.last_vision_time = None
        self.last_tag = -1

        # ------------------------------------------------------
        # Visual estimator
        # ------------------------------------------------------
        self.position_alpha = 0.40
        self.max_position_innovation = 0.25
        self.max_position_step = 0.025

        # Keep recent yaw by tag for stationary observations.
        self.yaw_history = deque(maxlen=100)

        # Vision is slow, so use a longer window and only 2
        # consistent samples for a stationary yaw estimate.
        self.yaw_window = 3.0
        self.yaw_cluster_radius = math.radians(5.0)
        self.min_yaw_cluster = 2

        # A yaw observation must be physically plausible for the
        # current segment to be used during translation/alignment.
        self.segment_yaw_gate = math.radians(20.0)

        # ------------------------------------------------------
        # Translation controller
        # ------------------------------------------------------
        self.drive_speed = 0.040
        self.near_speed = 0.025

        # Do not react to normal several-cm localization noise.
        self.cross_track_deadband = 0.060
        # Cross-track correction uses a bounded look-ahead geometry:
        # correction = atan2(-cross_track, lookahead).
        #
        # This is intentionally stronger than the earlier fixed gain
        # near the endpoint, while remaining bounded.
        self.cross_track_lookahead = 0.35  # m
        self.max_cross_heading = math.radians(12.0)

        # Heading thresholds.
        self.heading_deadband = math.radians(5.0)
        self.heading_correct = math.radians(7.0)

        # Endpoint logic.
        self.endpoint_remaining_tol = 0.070
        self.endpoint_distance_tol = 0.080
        self.max_waypoint_vision_age = 2.0
        self.max_drive_vision_age = 2.0
        self.max_near_drive_vision_age = 1.5

        # ------------------------------------------------------
        # Pulse turning
        # ------------------------------------------------------
        self.turn_cmd = 0.002
        self.turn_pulse_s = 0.10
        self.turn_rest_s = 1.80

        # Measured from 10-pulse calibration:
        # ~30 deg total -> ~3 deg / pulse.
        self.turn_per_pulse = math.radians(3.0)

        # When deciding that a stationary turn is complete.
        self.turn_done_tol = math.radians(3.5)

        # Pulse state.
        self.pulse_phase = "idle"   # idle, on, rest
        self.pulse_sign = 0.0
        self.phase_until = 0.0

        # Modes:
        # drive
        # align_segment
        # align_final
        # seek_tag2
        self.stage = "waiting_for_vision"

        self.last_v = 0.0
        self.last_w = 0.0

        self.dt = 0.05
        self.start_time = time.monotonic()
        self.last_status = 0.0

        # ------------------------------------------------------
        # Logs
        # ------------------------------------------------------
        log_dir = os.path.expanduser("~/hw2_logs")
        os.makedirs(log_dir, exist_ok=True)

        stamp = time.strftime("%Y%m%d_%H%M%S")

        self.traj_path = os.path.join(
            log_dir,
            f"trajectory_{stamp}.csv",
        )

        self.wp_path = os.path.join(
            log_dir,
            f"waypoints_{stamp}.csv",
        )

        self.traj_f = open(
            self.traj_path,
            "w",
            newline="",
        )

        self.traj = csv.writer(self.traj_f)

        self.traj.writerow([
            "time_s",
            "x_m",
            "y_m",
            "yaw_rad",
            "target_index",
            "stage",
            "cross_track_m",
            "remaining_m",
            "desired_heading_rad",
            "linear_cmd_mps",
            "angular_cmd_radps",
            "vision_age_s",
            "tag_id",
            "pulse_phase",
        ])

        self.wp_f = open(
            self.wp_path,
            "w",
            newline="",
        )

        self.wp_writer = csv.writer(self.wp_f)

        self.wp_writer.writerow([
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

        self.create_timer(
            self.dt,
            self.control,
        )

        self.create_timer(
            0.50,
            self.log_state,
        )

        self.get_logger().info(
            "Pulse-turn accuracy-first HW2 controller started."
        )

        self.get_logger().info(
            "Turn calibration: angular.z=0.002, "
            "0.10 s pulse ~= 3 deg, 1.80 s observe/rest."
        )

        self.get_logger().info(
            f"Trajectory log: {self.traj_path}"
        )

        self.get_logger().info(
            f"Waypoint log: {self.wp_path}"
        )

    # ==========================================================
    # Vision
    # ==========================================================

    def on_tag(self, msg):
        self.last_tag = int(msg.data)

    def on_vision(self, msg):
        now = self.get_clock().now()
        now_s = now.nanoseconds / 1e9

        m = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            q_to_yaw(msg.pose.orientation),
        ], dtype=float)

        tag = self.last_tag

        if tag >= 0:
            self.yaw_history.append(
                (now_s, tag, float(m[2]))
            )

        # ------------------------------------------------------
        # Known initial pose
        # ------------------------------------------------------
        if not self.have_vision:
            self.have_vision = True
            self.last_vision_time = now

            # Log measured localization error first.
            self.state = m.copy()

            self.record_waypoint(
                0,
                np.array([
                    0.0,
                    0.0,
                    0.0,
                ]),
            )

            self.get_logger().info(
                "Initial visual pose: "
                f"({m[0]:.3f}, {m[1]:.3f}, "
                f"{math.degrees(m[2]):.1f} deg)"
            )

            # Physical start is known exactly.
            self.state[:] = [
                0.0,
                0.0,
                0.0,
            ]

            if self.stop_after == 0:
                self.wp = len(self.points)
                self.stop_robot()
                return

            self.wp = 1
            self.stage = "drive"

            self.get_logger().info(
                "Anchored to known start (0,0,0); "
                "driving segment 1 straight."
            )

            return

        # ------------------------------------------------------
        # Position correction
        # ------------------------------------------------------
        # Freeze x,y during in-place turns.  The robot is not
        # translating, and PnP/tag-switch noise should not move the
        # estimated world position while it is physically stationary.
        if self.stage == "drive":
            e_xy = (
                m[:2]
                - self.state[:2]
            )

            innovation = float(
                np.linalg.norm(e_xy)
            )

            if (
                innovation
                <= self.max_position_innovation
            ):
                step = (
                    self.position_alpha
                    * e_xy
                )

                step_norm = float(
                    np.linalg.norm(step)
                )

                if step_norm > self.max_position_step:
                    step *= (
                        self.max_position_step
                        / step_norm
                    )

                self.state[0] += step[0]
                self.state[1] += step[1]

                self.last_vision_time = now

            else:
                self.get_logger().warn(
                    f"Rejected visual position innovation "
                    f"{innovation:.3f} m "
                    f"(tag {tag})"
                )
        else:
            # A fresh image/yaw observation still arrived, but x,y
            # remain fixed because this stage is an in-place turn.
            self.last_vision_time = now

    def latest_yaw_for_tag(
        self,
        tag_id,
        max_age=3.0,
    ):
        """
        Return the newest recent yaw observation for one tag.
        Used only to STOP the special tag-2 search immediately when
        tag 2 first enters view.
        """
        now_s = (
            self.get_clock().now().nanoseconds
            / 1e9
        )

        for t, tag, yaw in reversed(
            self.yaw_history
        ):
            if tag != tag_id:
                continue

            if now_s - t > max_age:
                continue

            return yaw

        return None

    def stable_yaw_for_tag(
        self,
        tag_id,
        target_yaw=None,
        gate=None,
    ):
        now_s = (
            self.get_clock().now().nanoseconds
            / 1e9
        )

        vals = []

        for t, tag, yaw in self.yaw_history:
            if tag != tag_id:
                continue

            if now_s - t > self.yaw_window:
                continue

            if (
                target_yaw is not None
                and gate is not None
                and abs(
                    wrap(
                        yaw - target_yaw
                    )
                ) > gate
            ):
                continue

            vals.append(yaw)

        if len(vals) < self.min_yaw_cluster:
            return None

        best = []

        for center in vals:
            cluster = [
                yaw
                for yaw in vals
                if abs(
                    wrap(
                        yaw - center
                    )
                )
                <= self.yaw_cluster_radius
            ]

            if len(cluster) > len(best):
                best = cluster

        if len(best) < self.min_yaw_cluster:
            return None

        return circ_mean(best)

    # ==========================================================
    # Geometry
    # ==========================================================

    def segment_geometry(self):
        start = self.points[
            self.wp - 1
        ]

        end = self.points[
            self.wp
        ]

        delta = end - start

        length = float(
            np.linalg.norm(delta)
        )

        tangent = (
            delta / length
        )

        left_normal = np.array([
            -tangent[1],
            tangent[0],
        ])

        rel = (
            self.state[:2]
            - start
        )

        along = float(
            np.dot(
                rel,
                tangent,
            )
        )

        cross = float(
            np.dot(
                rel,
                left_normal,
            )
        )

        remaining = (
            length - along
        )

        endpoint_distance = float(
            np.linalg.norm(
                end
                - self.state[:2]
            )
        )

        return (
            start,
            end,
            self.segment_heading[self.wp],
            cross,
            remaining,
            endpoint_distance,
        )

    def desired_heading_for_segment(
        self,
        nominal,
        cross,
        remaining,
    ):
        # Do not react inside the localization-noise deadband.
        if (
            abs(cross)
            <= self.cross_track_deadband
        ):
            correction = 0.0

        else:
            # Use a look-ahead correction.  Near the endpoint the
            # remaining distance naturally makes the required heading
            # correction more important, but cap it to avoid aggressive
            # turns from noisy position estimates.
            lookahead = max(
                0.20,
                min(
                    self.cross_track_lookahead,
                    max(remaining, 0.20),
                ),
            )

            correction = math.atan2(
                -cross,
                lookahead,
            )

            correction = max(
                -self.max_cross_heading,
                min(
                    self.max_cross_heading,
                    correction,
                ),
            )

        return wrap(
            nominal + correction
        )

    # ==========================================================
    # Helpers
    # ==========================================================

    def vision_age(self):
        if self.last_vision_time is None:
            return float("inf")

        return (
            self.get_clock().now()
            - self.last_vision_time
        ).nanoseconds / 1e9

    def publish(
        self,
        linear=0.0,
        angular=0.0,
    ):
        msg = Twist()

        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        self.cmd_pub.publish(msg)

        self.last_v = float(linear)
        self.last_w = float(angular)

    def stop_robot(self):
        self.publish(
            0.0,
            0.0,
        )

    def status(self, text):
        now = time.monotonic()

        if (
            now - self.last_status
            >= 1.0
        ):
            self.last_status = now
            self.get_logger().info(
                text
            )

    def record_waypoint(
        self,
        idx,
        target,
    ):
        px, py, yaw = target

        translation_error = math.hypot(
            px - self.state[0],
            py - self.state[1],
        )

        rotation_error = abs(
            wrap(
                yaw
                - self.state[2]
            )
        )

        self.wp_writer.writerow([
            idx,
            px,
            py,
            yaw,
            self.state[0],
            self.state[1],
            self.state[2],
            translation_error,
            rotation_error,
            time.monotonic()
            - self.start_time,
        ])

        self.wp_f.flush()

        self.get_logger().info(
            f"WAYPOINT {idx}: "
            f"pose=({self.state[0]:.3f}, "
            f"{self.state[1]:.3f}, "
            f"{math.degrees(self.state[2]):.1f} deg), "
            f"position_error={translation_error:.3f} m, "
            f"yaw_error={math.degrees(rotation_error):.1f} deg"
        )

    # ==========================================================
    # Pulse turn logic
    # ==========================================================

    def update_turn_pulse(
        self,
        desired_yaw,
        expected_tag=None,
        visual_gate=None,
    ):
        """
        Returns True when alignment is complete.
        Otherwise this method owns /cmd_vel while turning/resting.
        """
        now = time.monotonic()

        # ------------------------------------------------------
        # If resting, hold zero command and wait for observation.
        # ------------------------------------------------------
        if self.pulse_phase == "rest":
            self.stop_robot()

            if now < self.phase_until:
                return False

            # Rest finished. Use stable visual yaw if available.
            if expected_tag is not None:
                visual_yaw = self.stable_yaw_for_tag(
                    expected_tag,
                    desired_yaw,
                    visual_gate,
                )

                if visual_yaw is not None:
                    self.state[2] = visual_yaw

                    self.get_logger().info(
                        "Turn observation: "
                        f"tag={expected_tag}, "
                        f"yaw={math.degrees(visual_yaw):.1f} deg"
                    )

            self.pulse_phase = "idle"

        # ------------------------------------------------------
        # Active pulse.
        # ------------------------------------------------------
        if self.pulse_phase == "on":

            if now < self.phase_until:
                self.publish(
                    0.0,
                    self.pulse_sign
                    * self.turn_cmd,
                )

                return False

            # Pulse complete.
            self.stop_robot()

            # Update only a coarse internal prediction.
            self.state[2] = wrap(
                self.state[2]
                + self.pulse_sign
                * self.turn_per_pulse
            )

            self.pulse_phase = "rest"
            self.phase_until = (
                now
                + self.turn_rest_s
            )

            return False

        # ------------------------------------------------------
        # Idle: evaluate current yaw.
        # ------------------------------------------------------
        if expected_tag is not None:
            visual_yaw = self.stable_yaw_for_tag(
                expected_tag,
                desired_yaw,
                visual_gate,
            )

            if visual_yaw is not None:
                self.state[2] = visual_yaw

        error = wrap(
            desired_yaw
            - self.state[2]
        )

        self.status(
            "TURN: "
            f"target={math.degrees(desired_yaw):.1f}, "
            f"state={math.degrees(self.state[2]):.1f}, "
            f"error={math.degrees(error):.1f}"
        )

        if (
            abs(error)
            <= self.turn_done_tol
        ):
            self.stop_robot()
            return True

        # Start exactly one calibrated pulse.
        self.pulse_sign = (
            1.0
            if error > 0.0
            else -1.0
        )

        self.pulse_phase = "on"

        self.phase_until = (
            now
            + self.turn_pulse_s
        )

        self.publish(
            0.0,
            self.pulse_sign
            * self.turn_cmd,
        )

        return False

    def seek_tag2_left(self):
        """
        Special hard-coded transition after waypoint 2.

        Landmark geometry requires the rover to continue turning LEFT
        after finishing waypoint-2 yaw=180 deg until tag 2 is actually
        observed.  The coarse 3-deg/pulse prediction is NOT allowed to
        finish this stage by itself.

        Returns True only after a stable tag-2 yaw observation exists.
        """
        now = time.monotonic()

        # Stop the search on the FIRST fresh tag-2 observation.
        # With sub-1-Hz vision, waiting for two frames here could
        # rotate past the tag before the second image arrives.
        tag2_yaw = self.latest_yaw_for_tag(
            2,
            max_age=3.0,
        )

        if tag2_yaw is not None:
            self.stop_robot()
            self.state[2] = tag2_yaw
            self.pulse_phase = "idle"

            self.get_logger().info(
                "Tag 2 first acquired after waypoint 2: "
                f"visual yaw={math.degrees(tag2_yaw):.1f} deg. "
                "Stopping search and switching to "
                "return-segment alignment."
            )

            return True

        # During the rest phase, hold still and wait for vision.
        if self.pulse_phase == "rest":
            self.stop_robot()

            if now < self.phase_until:
                return False

            self.pulse_phase = "idle"

        # Finish an active pulse.
        if self.pulse_phase == "on":
            if now < self.phase_until:
                # Hard-coded LEFT turn only.
                self.publish(
                    0.0,
                    +self.turn_cmd,
                )
                return False

            self.stop_robot()

            # Coarse prediction only; never used to terminate seek.
            self.state[2] = wrap(
                self.state[2]
                + self.turn_per_pulse
            )

            self.pulse_phase = "rest"
            self.phase_until = (
                now
                + self.turn_rest_s
            )

            return False

        # Idle and still no tag 2: issue exactly one LEFT pulse.
        self.status(
            "SEEK TAG 2: no stable tag-2 visual pose yet; "
            "continuing one calibrated LEFT pulse."
        )

        self.pulse_sign = 1.0
        self.pulse_phase = "on"
        self.phase_until = (
            now
            + self.turn_pulse_s
        )

        self.publish(
            0.0,
            +self.turn_cmd,
        )

        return False

    # ==========================================================
    # Control
    # ==========================================================

    def control(self):
        if not self.have_vision:
            self.stop_robot()
            return

        if self.wp >= len(
            self.points
        ):
            self.stop_robot()
            return

        (
            _,
            _,
            nominal,
            cross,
            remaining,
            endpoint_distance,
        ) = self.segment_geometry()

        desired_heading = (
            self.desired_heading_for_segment(
                nominal,
                cross,
                remaining,
            )
        )

        expected_tag = self.segment_tag[
            self.wp
        ]

        # ------------------------------------------------------
        # DRIVE
        # ------------------------------------------------------
        if self.stage == "drive":

            # Never keep translating for a long interval without
            # a recent visual position update.  The rectified image
            # stream is intentionally treated as slow/intermittent.
            allowed_age = (
                self.max_near_drive_vision_age
                if remaining < 0.30
                else self.max_drive_vision_age
            )

            if self.vision_age() > allowed_age:
                self.stop_robot()

                self.status(
                    "Vision stale during translation: "
                    f"age={self.vision_age():.1f}s, "
                    f"limit={allowed_age:.1f}s. "
                    "Holding still for fresh visual position."
                )

                return

            # Update yaw from stationary/slow visual observations
            # only when physically plausible for this segment.
            visual_yaw = self.stable_yaw_for_tag(
                expected_tag,
                nominal,
                self.segment_yaw_gate,
            )

            if visual_yaw is not None:
                self.state[2] = visual_yaw

            heading_error = wrap(
                desired_heading
                - self.state[2]
            )

            # Endpoint reached.
            if (
                endpoint_distance
                <= self.endpoint_distance_tol
            ):
                self.stop_robot()

                if (
                    self.vision_age()
                    <= self.max_waypoint_vision_age
                ):
                    self.stage = "align_final"

                    self.pulse_phase = "idle"

                    self.get_logger().info(
                        f"Waypoint {self.wp} position reached: "
                        f"endpoint={endpoint_distance:.3f} m, "
                        f"remaining={remaining:.3f} m, "
                        f"cross={cross:.3f} m. "
                        "Starting final orientation."
                    )

                else:
                    self.status(
                        "Near waypoint but vision is stale; "
                        "holding still."
                    )

                return

            # Accuracy-first:
            # NEVER steer while translating.
            #
            # Stop if lateral or heading error is clearly outside
            # the measurement-noise deadband.
            # Cross-track error determines the desired heading.
            # It must NOT by itself trigger a stop.  If the robot is
            # already facing the corrective heading, it should keep
            # translating so that the lateral error can actually shrink.
            need_heading_correction = (
                abs(heading_error)
                >= self.heading_correct
            )

            if need_heading_correction:
                self.stop_robot()

                self.stage = "align_segment"
                self.pulse_phase = "idle"

                self.get_logger().info(
                    "Stopping for heading correction: "
                    f"cross={cross:.3f} m, "
                    f"heading_error="
                    f"{math.degrees(heading_error):.1f} deg, "
                    f"desired="
                    f"{math.degrees(desired_heading):.1f} deg"
                )

                return

            speed = self.drive_speed

            if remaining < 0.25:
                speed = self.near_speed

            # Straight translation only.
            self.publish(
                speed,
                0.0,
            )

            return

        # ------------------------------------------------------
        # SPECIAL RETURN TRANSITION AFTER WAYPOINT 2
        # ------------------------------------------------------
        if self.stage == "seek_tag2":

            found_tag2 = self.seek_tag2_left()

            if found_tag2:
                # wp is already 3 here.  Now that tag 2 provides a
                # trustworthy visual yaw, finish aligning to the fixed
                # return heading before any translation.
                self.stage = "align_segment"
                self.pulse_phase = "idle"

                self.get_logger().info(
                    "Preparing return to waypoint 0 using tag 2; "
                    f"target heading="
                    f"{math.degrees(self.segment_heading[3]):.1f} deg."
                )

            return

        # ------------------------------------------------------
        # SEGMENT ALIGNMENT / CROSS-TRACK CORRECTION
        # ------------------------------------------------------
        if self.stage == "align_segment":

            alignment_gate = (
                math.radians(90.0)
                if self.wp == 3
                else math.radians(45.0)
            )

            done = self.update_turn_pulse(
                desired_heading,
                expected_tag=expected_tag,
                visual_gate=alignment_gate,
            )

            if done:
                self.stage = "drive"

                self.get_logger().info(
                    "Segment heading corrected; "
                    "resuming straight translation."
                )

            return

        # ------------------------------------------------------
        # FINAL WAYPOINT ORIENTATION
        # ------------------------------------------------------
        if self.stage == "align_final":

            target_yaw = self.final_yaw[
                self.wp
            ]

            # For final orientation, try the current segment's tag
            # when it remains visible, but do not require it.
            done = self.update_turn_pulse(
                target_yaw,
                expected_tag=expected_tag,
                visual_gate=math.radians(45.0),
            )

            if not done:
                return

            target_pose = np.array([
                self.points[self.wp][0],
                self.points[self.wp][1],
                target_yaw,
            ])

            reached = self.wp

            self.record_waypoint(
                reached,
                target_pose,
            )

            if (
                reached
                >= self.stop_after
            ):
                self.get_logger().info(
                    f"Stopping after waypoint "
                    f"{reached}."
                )

                self.wp = len(
                    self.points
                )

                self.stop_robot()
                return

            self.wp += 1

            if self.wp >= len(
                self.points
            ):
                self.get_logger().info(
                    "All waypoints complete."
                )

                self.stop_robot()
                return

            self.pulse_phase = "idle"

            # Hard-coded landmark-layout transition:
            # after waypoint 2, keep turning LEFT until tag 2 is
            # actually visible.  Do not trust pulse dead reckoning
            # alone to start the return segment.
            if reached == 2 and self.wp == 3:
                self.stage = "seek_tag2"

                self.get_logger().info(
                    "Waypoint 2 complete. "
                    "Hard-coded return transition: keep turning LEFT "
                    "until tag 2 is visually acquired."
                )

            else:
                # Normal transition for other segments.
                self.stage = "align_segment"

                self.get_logger().info(
                    f"Preparing segment {self.wp}: "
                    f"nominal heading="
                    f"{math.degrees(self.segment_heading[self.wp]):.1f} deg."
                )

    # ==========================================================
    # Logging
    # ==========================================================

    def log_state(self):
        if (
            not self.have_vision
            or self.wp <= 0
            or self.wp >= len(self.points)
        ):
            return

        (
            _,
            _,
            nominal,
            cross,
            remaining,
            _,
        ) = self.segment_geometry()

        desired = (
            self.desired_heading_for_segment(
                nominal,
                cross,
                remaining,
            )
        )

        self.traj.writerow([
            time.monotonic()
            - self.start_time,
            self.state[0],
            self.state[1],
            self.state[2],
            self.wp,
            self.stage,
            cross,
            remaining,
            desired,
            self.last_v,
            self.last_w,
            self.vision_age(),
            self.last_tag,
            self.pulse_phase,
        ])

        self.traj_f.flush()

    def destroy_node(self):
        try:
            self.stop_robot()
        except Exception:
            pass

        try:
            self.traj_f.close()
            self.wp_f.close()
        except Exception:
            pass

        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--stop-after",
        type=int,
        default=3,
        choices=[
            0,
            1,
            2,
            3,
        ],
    )

    args = parser.parse_args()

    rclpy.init()

    node = HW2Controller(
        args.stop_after
    )

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
