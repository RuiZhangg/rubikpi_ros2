#!/usr/bin/env python3
"""Short, supervised ROS 2 motor tests for CSE 276A Homework 1.

Publish Float32MultiArray([left, right]) on /motor_commands.
The course's `motor_control` node MUST already be running; this script
never accesses /dev/ttyUSB0 itself.

Examples:
    python3 calibrate_motion.py forward 0.5 --speed 0.2
    python3 calibrate_motion.py left 0.2 --speed 0.5
    python3 calibrate_motion.py right 0.2 --speed 0.5
    python3 calibrate_motion.py custom 0.5 --left 0.19 --right 0.20
    python3 calibrate_motion.py stop 0.5
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

TOPIC = "/motor_commands"
PUBLISH_HZ = 25.0
MAX_COMMAND = 0.5  # Matches clipping in provided motor_control.py.
MAX_TEST_SECONDS = 10.0  # Initial supervised calibration only.


def read_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "motion", choices=("forward", "backward", "left", "right", "custom", "stop")
    )
    parser.add_argument("duration", type=float, help="Duration in seconds (0.1–5.0).")
    parser.add_argument(
        "--speed", type=float, default=None,
        help="Positive normalized magnitude; default 0.2 for driving, 0.5 for turns.",
    )
    parser.add_argument("--left", type=float, default=None, help="Custom left command.")
    parser.add_argument("--right", type=float, default=None, help="Custom right command.")
    args = parser.parse_args()

    if not math.isfinite(args.duration) or not 0.1 <= args.duration <= MAX_TEST_SECONDS:
        parser.error(f"duration must be between 0.1 and {MAX_TEST_SECONDS} seconds")

    if args.motion == "custom":
        if args.speed is not None:
            parser.error("Use --left/--right for custom mode, not --speed")
        if args.left is None or args.right is None:
            parser.error("custom mode requires both --left and --right")
        left, right = args.left, args.right
    else:
        if args.left is not None or args.right is not None:
            parser.error("--left/--right are only valid for custom mode")
        if args.motion == "stop":
            if args.speed is not None:
                parser.error("stop mode does not take --speed")
            left, right = 0.0, 0.0
        else:
            speed = args.speed if args.speed is not None else (
                0.2 if args.motion in ("forward", "backward") else 0.5
            )
            if not math.isfinite(speed) or not 0 < speed <= MAX_COMMAND:
                parser.error(f"--speed must be > 0 and <= {MAX_COMMAND}")
            left, right = {
                "forward": (speed, speed),
                "backward": (-speed, -speed),
                "left": (-speed, speed),
                "right": (speed, -speed),
            }[args.motion]

    if not all(math.isfinite(value) and abs(value) <= MAX_COMMAND for value in (left, right)):
        parser.error(f"motor commands must be finite and in [-{MAX_COMMAND}, {MAX_COMMAND}]")
    return args, float(left), float(right)


def publish(publisher, left, right):
    msg = Float32MultiArray()
    msg.data = [float(left), float(right)]
    publisher.publish(msg)


def publish_for(publisher, left, right, duration):
    """Repeatedly publish at 25 Hz; return actual elapsed publishing time."""
    period = 1.0 / PUBLISH_HZ
    start = time.monotonic()
    next_send = start
    while time.monotonic() - start < duration:
        publish(publisher, left, right)
        next_send += period
        time.sleep(max(0.0, min(period, next_send - time.monotonic())))
    return time.monotonic() - start


def main():
    args, left, right = read_args()
    rclpy.init()
    node = Node("calibrate_motion_node")
    publisher = node.create_publisher(Float32MultiArray, TOPIC, 10)
    exit_code = 0
    try:
        print("Waiting for the existing motor_control subscriber...")
        deadline = time.monotonic() + 6.0
        while node.count_subscribers(TOPIC) < 1 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.count_subscribers(TOPIC) < 1:
            raise RuntimeError(
                "No motor controller subscribes to /motor_commands. "
                "Start 'ros2 run robot_control motor_control' in another terminal "
                "and check ROS_DOMAIN_ID."
            )

        # Give DDS discovery a little time; send only zero commands during warm-up.
        publish_for(publisher, 0.0, 0.0, 0.3)
        print(
            f"RUN motion={args.motion}, left={left:+.3f}, right={right:+.3f}, "
            f"requested={args.duration:.3f}s, rate={PUBLISH_HZ:.0f}Hz",
            flush=True,
        )
        actual = publish_for(publisher, left, right, args.duration)
        print(f"Finished publishing after {actual:.3f}s. STOP commands follow.")
    except KeyboardInterrupt:
        print("Interrupted: sending stop commands.")
        exit_code = 130
    except Exception as exc:
        print(f"ERROR: {exc}")
        exit_code = 1
    finally:
        # Repeated stops are safer than one packet; bridge has its own watchdog.
        try:
            publish_for(publisher, 0.0, 0.0, 0.6)
        except Exception as exc:
            print(f"Warning: could not complete stop publishing: {exc}")
            exit_code = exit_code or 1
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
