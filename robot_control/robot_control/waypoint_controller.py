#!/usr/bin/env python3
"""CSE 276A HW1: calibrated open-loop waypoint navigation.

This program is intentionally SAFE BY DEFAULT: without --execute it only
prints the planned commands. Dry-run works on a Mac without ROS installed.

It does not open the USB serial port. The course's existing
    ros2 run robot_control motor_control
must be running on the Rubik Pi for --execute.

Examples (from the ROS package root):
    python3 robot_control/waypoint_controller.py --dry-run
    python3 robot_control/waypoint_controller.py --dry-run --segments 2
    python3 robot_control/waypoint_controller.py --execute --segments 1 --hardware-checked
    python3 robot_control/waypoint_controller.py --execute --hardware-checked --allow-unverified-right

CAUTION: Do NOT execute physical turns if any wheel is stalling or binding.
Latest turn calibration followed an improvised painter-tape alteration near
an intermittently failing right-rear wheel. This does NOT verify hardware safety.
Optional turn-duration anchors are interpolated; motion still remains open-loop.
Turn-center translation compensation can be estimated from measured route
center-position changes. The current config contains a fitted effective-pivot
model and should be verified; it is not live localization.
Right-turn timing has been measured in the latest altered configuration, but
that does not establish drivetrain safety.
"""

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_WAYPOINTS = PACKAGE_ROOT / "config" / "waypoints.txt"
DEFAULT_CALIBRATION = PACKAGE_ROOT / "config" / "calibration.json"
TOPIC = "/motor_commands"
MAX_COMMAND = 0.5
MAX_SINGLE_MOTION_SEC = 8.0  # Per-motion guard; may be changed deliberately in source.
ANGLE_EPS = math.radians(3.0)  # suppress tiny, uncalibrated pulses on ~20Hz hardware
DISTANCE_EPS = 0.001


@dataclass(frozen=True)
class Pose:
    x: float                 # meters in the scaled map
    y: float                 # meters in the scaled map
    theta: float             # radians, positive CCW


@dataclass(frozen=True)
class Motion:
    leg: int                 # 1 means waypoint 0 -> waypoint 1
    kind: str                # 'left', 'right', or 'forward'
    amount: float            # radians for turns; meters for forward
    duration: float          # seconds
    left: float              # normalized motor input
    right: float             # normalized motor input


def wrap_angle(angle):
    """Shortest signed angular displacement in [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def finite_number(value, description):
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{description} must be a number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{description} must be finite")
    return number


def load_config(path):
    with open(path, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    scale = finite_number(cfg.get("scale_m_per_unit"), "scale_m_per_unit")
    if scale <= 0:
        raise ValueError("scale_m_per_unit must be positive")
    cfg["scale_m_per_unit"] = scale

    for name, speed_key in (
        ("forward", "meters_per_second"),
        ("left", "degrees_per_second"),
        ("right", "degrees_per_second"),
    ):
        section = cfg.get(name)
        if not isinstance(section, dict):
            raise ValueError(f"Missing '{name}' object in calibration.json")
        commands = section.get("commands")
        if not isinstance(commands, list) or len(commands) != 2:
            raise ValueError(f"{name}.commands must contain exactly [left, right]")
        section["commands"] = [
            finite_number(commands[0], name + " left command"),
            finite_number(commands[1], name + " right command"),
        ]
        if any(abs(c) > MAX_COMMAND for c in section["commands"]):
            raise ValueError(f"{name}.commands must be within [-0.5, 0.5]")
        speed = finite_number(section.get(speed_key), name + "." + speed_key)
        if speed <= 0:
            raise ValueError(f"{name}.{speed_key} must be positive")
        section[speed_key] = speed
        # Optional angle-duration anchors from actual trials. The legacy
        # degrees_per_second field remains the backward-compatible fallback.
        if name in ("left", "right") and "duration_curve_deg_sec" in section:
            raw = section["duration_curve_deg_sec"]
            if not isinstance(raw, list) or len(raw) < 2:
                raise ValueError(f"{name}.duration_curve_deg_sec requires >= 2 [angle_deg,time_s] points")
            points = []
            for i, pair in enumerate(raw):
                if not isinstance(pair, list) or len(pair) != 2:
                    raise ValueError(f"{name} curve point {i} must be [angle_degrees, duration_seconds]")
                a = finite_number(pair[0], f"{name} curve angle {i}")
                t = finite_number(pair[1], f"{name} curve duration {i}")
                if a < 0 or t < 0:
                    raise ValueError(f"{name} curve points must be nonnegative")
                if points and (a <= points[-1][0] or t <= points[-1][1]):
                    raise ValueError(f"{name} curve angles and times must strictly increase")
                points.append((a, t))
            if points[0] != (0.0, 0.0):
                raise ValueError(f"{name} curve must start with [0,0]")
            section["duration_curve_deg_sec"] = points
    cfg["publish_hz"] = finite_number(cfg.get("publish_hz", 25.0), "publish_hz")
    # Optional rotation-induced chassis translation model.  Coordinates are
    # measured in the robot body frame BEFORE the turn: x forward, y left.
    # The robot may pivot around a point away from its geometric center.
    shift = cfg.setdefault("turn_center_shift", {"enabled": False})
    if not isinstance(shift, dict):
        raise ValueError("turn_center_shift must be a JSON object")
    enabled = shift.get("enabled", False)
    if not isinstance(enabled, bool):
        raise ValueError("turn_center_shift.enabled must be true or false")
    for name in ("left_90deg_body_m", "right_90deg_body_m"):
        raw = shift.get(name)
        if raw is None:
            if enabled:
                raise ValueError(f"Turn compensation enabled but {name} is missing; measure both dx and dy")
            continue
        if not isinstance(raw, list) or len(raw) != 2:
            raise ValueError(f"turn_center_shift.{name} must be [forward_dx_m, left_dy_m] or null")
        shift[name] = [finite_number(raw[0], name + " dx"), finite_number(raw[1], name + " dy")]
        if math.hypot(*shift[name]) > 0.5:
            raise ValueError(f"{name} magnitude exceeds 0.5m; check measurement units")
    cfg["settle_sec"] = finite_number(cfg.get("settle_sec", 0.30), "settle_sec")
    if not 15 <= cfg["publish_hz"] <= 50:
        raise ValueError("publish_hz must be in [15, 50] Hz")
    if not 0 <= cfg["settle_sec"] <= 2:
        raise ValueError("settle_sec must be in [0, 2] seconds")
    return cfg


def load_waypoints(path, scale):
    poses = []
    with open(path, "r", encoding="utf-8") as f:
        for line_number, original in enumerate(f, 1):
            line = original.split("#", 1)[0].strip()
            if not line:
                continue
            fields = [p.strip() for p in line.split(",")]
            if len(fields) != 3:
                raise ValueError(f"{path}:{line_number}: expected x,y,theta")
            x, y, theta = [
                finite_number(value, f"waypoint line {line_number}")
                for value in fields
            ]
            poses.append(Pose(scale * x, scale * y, theta))
    if len(poses) < 2:
        raise ValueError("At least two waypoints are needed")
    return poses


def make_motion(leg, kind, amount, cfg):
    if kind == "forward":
        duration = amount / cfg[kind]["meters_per_second"]
    else:
        angle = abs(math.degrees(amount))
        curve = cfg[kind].get("duration_curve_deg_sec")
        if curve:
            if angle > curve[-1][0] + 1e-8:
                raise ValueError(f"{kind} angle {angle:.2f} deg exceeds measured curve "+
                                 f"maximum {curve[-1][0]:.2f} deg; avoid unvalidated extrapolation")
            # Between anchors, interpolate the motor-on time. These are
            # empirical pose-change results, not instantaneous angular speed.
            duration = None
            for (a0, t0), (a1, t1) in zip(curve[:-1], curve[1:]):
                if a0 <= angle <= a1 + 1e-8:
                    duration = t0 + (angle - a0) * (t1 - t0) / (a1 - a0)
                    break
            if duration is None:
                raise ValueError(f"No calibrated interval for {kind} turn of {angle:.2f} deg")
        else:
            duration = angle / cfg[kind]["degrees_per_second"]
    left, right = cfg[kind]["commands"]
    if not math.isfinite(duration) or not 0 < duration <= MAX_SINGLE_MOTION_SEC:
        raise ValueError(
            f"leg {leg} {kind}: computed duration {duration:.3f} s "
            f"outside (0, {MAX_SINGLE_MOTION_SEC}] s. Check calibration."
        )
    return Motion(leg, kind, amount, duration, left, right)


def rotate_xy(x, y, theta):
    """Body-frame 2D vector -> world-frame vector at heading theta."""
    c, s = math.cos(theta), math.sin(theta)
    return (c*x - s*y, s*x + c*y)


def turn_world_displacement(theta_before, turn_angle, cfg):
    """Predict chassis-center translation while rotating (NOT measured pose).

    Constant-effective-pivot approximation. Estimate pivot location c in
    the body frame using equivalent 90-degree shifts fitted from several measured turns:
        d(delta) = (I - R(delta)) * c.
    Works exactly at the 90deg calibration anchor; extrapolations, especially
    near 180deg, are UNVERIFIED until additional experiments.
    """
    shift = cfg.get("turn_center_shift", {})
    if not shift.get("enabled") or abs(turn_angle) < ANGLE_EPS:
        return (0.0, 0.0)
    left = turn_angle > 0
    key = "left_90deg_body_m" if left else "right_90deg_body_m"
    dx90, dy90 = shift[key]
    sign = 1.0 if left else -1.0
    # Solve (I-R(sign*pi/2))*c = d90.
    cx = (dx90 - sign*dy90)/2.0
    cy = (sign*dx90 + dy90)/2.0
    cost, sint = math.cos(turn_angle), math.sin(turn_angle)
    dx_body = (1-cost)*cx + sint*cy
    dy_body = -sint*cx + (1-cost)*cy
    return rotate_xy(dx_body, dy_body, theta_before)


def predict_after(pose, motion, cfg):
    """Model-predicted pose after a motion; NOT sensor feedback."""
    if motion.kind == "forward":
        dx, dy = rotate_xy(motion.amount, 0.0, pose.theta)
        return Pose(pose.x + dx, pose.y + dy, pose.theta)
    dx, dy = turn_world_displacement(pose.theta, motion.amount, cfg)
    return Pose(pose.x + dx, pose.y + dy, wrap_angle(pose.theta + motion.amount))


def compensated_leg(start, target, cfg, leg):
    """Plan turn/translate/turn accounting for predicted turn-induced shifts.

    Solve for a travel heading so that after the first turn and the final
    orientation turn, the predicted center is at the target. Since both turns
    translate the chassis, the required drive endpoint differs from target.
    A fixed-point geometric iteration finds an approximate solution.
    """
    dx, dy = target.x - start.x, target.y - start.y
    if math.hypot(dx, dy) <= DISTANCE_EPS:
        delta = wrap_angle(target.theta-start.theta)
        return [] if abs(delta) <= ANGLE_EPS else [make_motion(leg, 'left' if delta > 0 else 'right', delta, cfg)]
    heading = math.atan2(dy, dx)
    for _ in range(80):
        first_turn = wrap_angle(heading-start.theta)
        first_dx,first_dy = turn_world_displacement(start.theta, first_turn, cfg)
        last_turn = wrap_angle(target.theta-heading)
        last_dx,last_dy = turn_world_displacement(heading, last_turn, cfg)
        drive_dx = target.x-last_dx-(start.x+first_dx)
        drive_dy = target.y-last_dy-(start.y+first_dy)
        if math.hypot(drive_dx,drive_dy) < DISTANCE_EPS:
            raise ValueError(f'leg {leg}: turn shifts consume the drive distance')
        new_heading=math.atan2(drive_dy,drive_dx)
        if abs(wrap_angle(new_heading-heading)) < 1e-10:
            heading=new_heading
            break
        # Gentle damping helps prevent oscillation when offsets are sizable.
        heading=wrap_angle(heading+0.5*wrap_angle(new_heading-heading))
    else:
        raise ValueError(f'leg {leg}: shift-compensated heading did not converge')

    first_turn=wrap_angle(heading-start.theta)
    first_dx,first_dy=turn_world_displacement(start.theta,first_turn,cfg)
    last_turn=wrap_angle(target.theta-heading)
    last_dx,last_dy=turn_world_displacement(heading,last_turn,cfg)
    drive_dx=target.x-last_dx-start.x-first_dx
    drive_dy=target.y-last_dy-start.y-first_dy
    distance=math.hypot(drive_dx,drive_dy)
    actions=[]
    if abs(first_turn)>ANGLE_EPS:
        actions.append(make_motion(leg,'left' if first_turn>0 else 'right',first_turn,cfg))
    if distance>DISTANCE_EPS:
        actions.append(make_motion(leg,'forward',distance,cfg))
    if abs(last_turn)>ANGLE_EPS:
        actions.append(make_motion(leg,'left' if last_turn>0 else 'right',last_turn,cfg))
    return actions


def plan_route(poses, cfg, segment_limit=None):
    """Plan from waypoints; optional calibrated turn-translation compensation.

    The geometric prediction is never a measured pose; if the wheels slip or
    stall, errors still accumulate and will not be detected by this planner.
    """
    leg_count = len(poses) - 1
    if segment_limit is not None:
        if not 1 <= segment_limit <= leg_count:
            raise ValueError(f"--segments must be between 1 and {leg_count}")
        leg_count = segment_limit
    actions = []
    estimate = poses[0]
    for leg in range(1, leg_count + 1):
        target = poses[leg]
        leg_actions = compensated_leg(estimate, target, cfg, leg)
        actions.extend(leg_actions)
        for motion in leg_actions:
            estimate = predict_after(estimate, motion, cfg)
        if (cfg.get('turn_center_shift',{}).get('enabled')
                and math.hypot(estimate.x-target.x,estimate.y-target.y)>0.005):
            raise ValueError(f'leg {leg}: predicted compensated endpoint deviates >5 mm from target')
        # Without compensation, original pure turn/drive/turn model is used.
        # With compensation, 'estimate' includes experimentally modeled shifts.
    return actions, leg_count


def show_plan(poses, actions, leg_count, cfg, waypoint_path, config_path):
    print("\nCSE 276A HW1 — OPEN-LOOP PLAN (predictions, not actual positions)")
    print(f"Waypoints : {waypoint_path}")
    print(f"Calibration: {config_path}")
    print(f"Scale     : {cfg['scale_m_per_unit']:.3f} m/coordinate unit")
    print("Initial   : manually align robot to waypoint 0 before execution")
    shift_enabled = cfg.get("turn_center_shift",{}).get("enabled",False)
    print(f"Turn-center compensation: {'ON (empirical approximate)' if shift_enabled else 'OFF (not yet measured)'}")
    print(f"Segments  : {leg_count} / {len(poses)-1}")
    estimate = poses[0]
    predicted_centers = [(estimate.x, estimate.y)]
    for leg in range(1, leg_count + 1):
        p = poses[leg]
        print(f"\nWaypoint {leg}: predicted pose ({p.x:.3f}, {p.y:.3f}, {p.theta:.3f} rad)")
        movements = [m for m in actions if m.leg == leg]
        if not movements:
            print("  no motor movement (same pose)")
        for motion in movements:
            unit = "m" if motion.kind == "forward" else "deg"
            amount = motion.amount if motion.kind == "forward" else math.degrees(motion.amount)
            print(
                f"  {motion.kind:<7} {amount:+8.2f} {unit:<3}  "
                f"{motion.duration:5.2f} s  [L,R]=[{motion.left:+.3f},{motion.right:+.3f}]"
            )
            estimate = predict_after(estimate, motion, cfg)
            predicted_centers.append((estimate.x, estimate.y))
            print(f"      predicted center after action: ({estimate.x:.3f}, {estimate.y:.3f}) m, heading={math.degrees(estimate.theta):+.1f} deg")
        print(f"  model endpoint: ({estimate.x:.3f}, {estimate.y:.3f}, {estimate.theta:.3f} rad) — NOT measured")
    # Print envelope warnings for all intermediate predicted action endpoints.
    # These are mathematical predictions and cannot guarantee actual clearance.
    xmin = min(p.x for p in poses); xmax = max(p.x for p in poses)
    ymin = min(p.y for p in poses); ymax = max(p.y for p in poses)
    outside = [(x,y) for x,y in predicted_centers
               if x < xmin-0.001 or x > xmax+0.001 or y < ymin-0.001 or y > ymax+0.001]
    if outside:
        print(f"WARNING: {len(outside)} predicted action centers OUTSIDE waypoint bounding rectangle "
              f"x=[{xmin:.2f},{xmax:.2f}], y=[{ymin:.2f},{ymax:.2f}] m.")
        print("Do NOT attempt the complete motion in a tight workspace without inspecting the action centers and clearance.")
    else:
        print("Predicted action centers remain inside nominal waypoint bounding rectangle; "
              "real centers and robot footprint may still exceed the safe area.")
    print(f"\nTotal translation: {sum(a.amount for a in actions if a.kind == 'forward'):.3f} m")
    print(f"Total moving time: {sum(a.duration for a in actions):.1f} s (plus stops/settling)")
    if any(a.kind == "right" for a in actions):
        tested = cfg["right"].get("timing_tested_current_configuration", cfg["right"].get("verified_current_hardware", False))
        if tested:
            print("Right-turn TIMING was measured in current altered configuration; drivetrain safety is NOT thereby verified.")
        else:
            print("WARNING: RIGHT-TURN TIMING IS UNVERIFIED IN THE CURRENT CONFIGURATION.")
    if not shift_enabled:
        print("WARNING: observed turn shifts are NOT compensated; measure signed x/y per turn.")
    else:
        print("WARNING: shift model is fitted from 4 measured turn actions (including a ~180deg turn); it is approximate and needs physical validation.")
    print("WARNING: right-rear wheel fault / tape alteration has not been mechanically cleared.")
    print("WARNING: No encoder/vision feedback. Predicted pose may differ from real pose.")


def execute(actions, cfg, pause_at_waypoints=False, pause_after_actions=False):
    """Only invoked after explicit --execute and safety checks."""
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32MultiArray
    except ImportError as exc:
        raise RuntimeError("Execution requires ROS 2 Jazzy Python on the Rubik Pi") from exc

    rclpy.init(args=[])
    node = Node("hw1_waypoint_controller")
    pub = node.create_publisher(Float32MultiArray, TOPIC, 10)
    period = 1.0 / cfg["publish_hz"]

    def publish(left, right):
        msg = Float32MultiArray()
        msg.data = [float(left), float(right)]
        pub.publish(msg)

    def send_for(left, right, seconds):
        start = time.monotonic()
        deadline = start + seconds
        next_send = start
        while time.monotonic() < deadline:
            if not rclpy.ok():
                raise RuntimeError("ROS shut down while sending commands")
            publish(left, right)
            next_send += period
            time.sleep(max(0.0, min(period, next_send - time.monotonic())))

    def stop():
        # Repeated zero messages, rather than a single zero packet.
        send_for(0.0, 0.0, 0.35)

    try:
        print("Waiting for existing /motor_commands subscriber (motor_control node)...", flush=True)
        wait_end = time.monotonic() + 6.0
        while node.count_subscribers(TOPIC) < 1 and time.monotonic() < wait_end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.count_subscribers(TOPIC) < 1:
            raise RuntimeError("Motor-controller subscriber not found; no movement sent")
        send_for(0.0, 0.0, 0.35)
        for i, a in enumerate(actions, 1):
            print(f"Action {i}/{len(actions)}: leg={a.leg} {a.kind} "
                  f"time={a.duration:.3f}s L={a.left:+.3f} R={a.right:+.3f}", flush=True)
            try:
                send_for(a.left, a.right, a.duration)
            finally:
                stop()
            if cfg["settle_sec"] > 0:
                send_for(0.0, 0.0, cfg["settle_sec"])
            # Optional checkpoint after each completed waypoint leg.  The
            # bridge's watchdog will maintain stop while the user inspects.
            # This does NOT update the predicted pose or steer the rover.
            is_leg_boundary = i == len(actions) or actions[i].leg != a.leg
            if (pause_after_actions or (pause_at_waypoints and is_leg_boundary)) and i < len(actions):
                detail = f"action {i}/{len(actions)} ({a.kind}, waypoint {a.leg})" if pause_after_actions else f"waypoint {a.leg}"
                print(f"Completed planned {detail}; robot is STOPPED. ", end="", flush=True)
                try:
                    choice = input("Inspect / measure, Enter to continue, q to abort: ").strip().lower()
                except EOFError as exc:
                    raise RuntimeError("Input unavailable at waypoint pause; remaining commands canceled") from exc
                if choice in ("q", "quit", "stop"):
                    raise RuntimeError("Operator aborted at waypoint checkpoint; motors stopped")
        print("Movement sequence completed (actual pose NOT measured).", flush=True)
    finally:
        try:
            stop()
        except Exception as exc:
            print(f"WARNING: could not complete final zero-command stream: {exc}", file=sys.stderr)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--waypoints", type=Path, default=DEFAULT_WAYPOINTS)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--scale", type=float, default=None,
                        help="Optional meters per coordinate unit; overrides config for this run (default from JSON)")
    parser.add_argument("--workspace", type=float, nargs=4, metavar=("XMIN", "YMIN", "XMAX", "YMAX"),
                        help="Actual safe robot-CENTER region relative to initial robot center, in meters; checks predicted action endpoints")
    parser.add_argument("--segments", type=int, default=None,
                        help="Number of waypoint-to-waypoint legs to plan/run (default: all)")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="Print plan only (default)")
    mode.add_argument("--execute", action="store_true", help="Actually drive robot")
    parser.add_argument("--hardware-checked", action="store_true",
                        help="Explicit acknowledgement wheels/drivetrain have been checked")
    parser.add_argument("--allow-unverified-right", action="store_true",
                        help="Override lack of right-turn TIMING data (does not certify drivetrain safety)")
    pauses = parser.add_mutually_exclusive_group()
    pauses.add_argument("--pause-at-waypoints", action="store_true",
                        help="STOP after each waypoint and wait for Enter")
    pauses.add_argument("--pause-after-actions", action="store_true",
                        help="STOP after EVERY turn/drive action to measure the individual motion")
    args = parser.parse_args()

    try:
        cfg = load_config(args.calibration)
        if args.scale is not None:
            if not math.isfinite(args.scale) or not 0 < args.scale <= 1.0:
                raise ValueError("--scale must be finite and in (0,1]")
            cfg["scale_m_per_unit"] = args.scale
        poses = load_waypoints(args.waypoints, cfg["scale_m_per_unit"])
        actions, count = plan_route(poses, cfg, args.segments)
        show_plan(poses, actions, count, cfg, args.waypoints, args.calibration)
        if args.workspace is not None:
            xmin, ymin, xmax, ymax = args.workspace
            if not all(math.isfinite(z) for z in args.workspace) or xmin >= xmax or ymin >= ymax:
                raise ValueError("--workspace requires finite XMIN<XMAX and YMIN<YMAX")
            trace_pose = poses[0]
            trace = [(0, trace_pose)]
            for i, movement in enumerate(actions, 1):
                trace_pose = predict_after(trace_pose, movement, cfg)
                trace.append((i, trace_pose))
            risky = [(i, p) for i,p in trace
                     if not (xmin <= p.x <= xmax and ymin <= p.y <= ymax)]
            if risky:
                print("WORKSPACE CENTER-POINT LIMIT VIOLATIONS: " + ", ".join(
                    f"after action {i}: ({p.x:.3f},{p.y:.3f})" for i,p in risky))
                if args.execute:
                    raise ValueError("Predicted action center exceeds --workspace. Change physical origin/scale or do not run; actual errors and robot footprint require MORE margin.")
            else:
                print("Workspace center-point prediction: all action endpoints within specified bounds; this DOES NOT certify actual clearance or robot footprint.")
        if not args.execute:
            print("\nDRY RUN ONLY. No ROS messages sent; motors will NOT move.")
            return 0
        if not args.hardware_checked:
            parser.error("physical execution requires --hardware-checked; DO NOT run with a stalling wheel")
        right_timing_tested = cfg["right"].get("timing_tested_current_configuration", cfg["right"].get("verified_current_hardware", False))
        unverified_right = any(a.kind == "right" for a in actions) and not right_timing_tested
        if unverified_right and not args.allow_unverified_right:
            parser.error("right-turn timing is not measured in current configuration; "
                         "recalibrate or use --allow-unverified-right for unverified timing")
        print("\nEXECUTION ENABLED: Stay next to robot and ready to cut motor power.", flush=True)
        if args.pause_at_waypoints or args.pause_after_actions:
            print("PAUSES ENABLED: Enter to resume, q to stop. DO NOT reposition the robot during a continuous trial.", flush=True)
        execute(actions, cfg, pause_at_waypoints=args.pause_at_waypoints,
                pause_after_actions=args.pause_after_actions)
        return 0
    except KeyboardInterrupt:
        print("Interrupted. The ROS execution path attempts to send repeated STOP commands.", file=sys.stderr)
        return 130
    except (ValueError, FileNotFoundError, RuntimeError, OSError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
