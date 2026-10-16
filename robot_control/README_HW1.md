# CSE 276A — Homework 1: Open-Loop Waypoint Navigation

**Student:** Rui Zhang
**Due:** October 16, 2026  
**Report status:** Draft — replace all `[TODO]` fields with real experimental results before submitting.  
**Video:** [TODO: unlisted YouTube URL]

> **Submission:** This Markdown is the editable report source and the project README. Export a final `report.pdf` from it and include both the PDF and the runnable `robot_control` package in the ZIP.

## 1. Objective and experiment setup

The goal is to traverse the assigned sequence of planar poses \((x,y,\theta)\) autonomously using Python and ROS 2 on a four-wheel skid-steering robot. We approximate the vehicle as differential drive and use open-loop, time-based motor commands. 

- **Platform:** Wave Rover + Rubik Pi; Ubuntu 24.04, ROS 2 Jazzy.
- **Controller architecture:** `waypoint_controller.py` (to be implemented) publishes `std_msgs/msg/Float32MultiArray` messages on `/motor_commands`; the provided `motor_control.py` forwards them over USB serial (`/dev/ttyUSB0`) to the rover's motor controller.
- **Coordinate convention:** world +x right/forward at the starting pose, +y 90° counterclockwise, yaw positive CCW, yaw angles in radians.
- **Starting pose assumption:** manually place robot center at `(0,0)`, yaw `0` before every full run.
- **Physical scale:** 1 coordinate unit in the provided file corresponds to **0.8 m**, following instructor's permission for a scaled workspace. 
- **Test surface:** dorm's low carpet, due to best accessibility
- **Usable test area:** about 1.6 m × 1.6 m, with robot clearance already considered.
- **Measurement method:** Rolling ruler with set square rulers and protractor

### Input waypoints

The source file `config/waypoints.txt` contains the original values, without modifying them:

```text
0,0,0
1,0,0
1,1,1.57
2,1,0
2,2,-1.57
1,1,-0.78
0,0,0
```

Positions are multiplied by `scale = 0.8 m per unit` at runtime. The scaled path spans \(x,y\in[0,1.6]\) m. Rotations add no ideal translational distance. Each target includes a required **final heading**, not merely an (x,y) position.

## 2. Kinematic model and assumptions

For ideal differential drive with effective left/right longitudinal wheel speeds \(v_L\) and \(v_R\), and effective track width \(b\):

\[
 v = \frac{v_R+v_L}{2}, \qquad \omega = \frac{v_R-v_L}{b}.
\]

For planar pose \((x,y,\theta)\):

\[
 \dot x = v\cos\theta,\quad \dot y = v\sin\theta,\quad \dot\theta = \omega.
\]

The rover uses four-wheel skid steering, so the ideal no-slip differential-drive equations are approximations. **The ROS motor inputs are normalized command values, not measured wheel speeds or meters/second.** Instead of assuming a known wheel radius and track width, our implemented controller uses an **empirical command-to-motion model** for a selected set of constant inputs:

- Forward command `([TODO L], [TODO R])`: \(d\approx \hat v_f t\), with \(\hat v_f\) in m/s.
- Left in-place turn `([TODO L], [TODO R])`: \(\Delta\theta\approx\hat\omega_L t\), \(\hat\omega_L>0\).
- Right in-place turn `([TODO L], [TODO R])`: \(\Delta\theta\approx\hat\omega_R t\), \(\hat\omega_R<0\).

These approximations assume reasonably repeatable motor response, floor friction, battery condition, and motion intervals. Short motions may show startup delay or acceleration effects; if measurements support it, use an empirical intercept or a duration lookup rather than assuming perfect linearity.

**Important open-loop limitation:** Predicted pose is updated using the requested displacement and rotation, **not** an observed pose. Slip and timing error therefore accumulate across the path.

## 3. Calibration method and results — 4-point rubric category

**Procedure:** Mark a reference origin, axis, and headings on the floor. Start each trial from the same pose. Send a motor command at 25 Hz for a specified interval, then publish repeated zero commands. After the robot stops, measure actual displacement and/or angular change. Store all trials in `calibration/trials.csv` (including command values, duration, measured result, and notes).

### Translation calibration

| Command [L,R] | Duration (s) | Trial distance (m) | Heading drift (deg) | Notes |
|---|---:|---:|---:|---|
| `[0.20,0.20]` | 1.0 | [TODO] | [TODO] | Trial 3 |
| `[0.20,0.20]` | 1.5 | [TODO] | [TODO] | Trial 4 |
| `[0.20,0.20]` | 3.0 | [TODO] | [TODO] | Check longer-motion behavior |

Estimate an effective forward velocity from the measured \(d_i,t_i\) values, e.g. \(\hat v_f\approx\sum_i d_i/\sum_i t_i\) as a simple initial model. Discuss whether a through-origin proportional model is justified. **Selected \(\hat v_f\): [TODO] m/s.**

### Rotation calibration

| Direction | Command [L,R] | Duration (s) | Measured rotation (deg) | Notes |
|---|---|---:|---:|---|
| Left | `[-0.50,+0.50]` | 0.2 | [TODO] | Trial 1 |
| Left | `[-0.50,+0.50]` | 0.3 | [TODO] | Trial 2 |
| Right | `[+0.50,-0.50]` | 0.2 | [TODO] | Trial 1 |
| Right | `[+0.50,-0.50]` | 0.3 | [TODO] | Trial 2 |
| Left | chosen | [TODO] | [TODO] | Predicted 90° confirmation |
| Right | chosen | [TODO] | [TODO] | Predicted 90° confirmation |
| Either | chosen | [TODO] | [TODO] | Predicted 180° confirmation |

Convert measured angle to radians, then estimate angular velocity with \(\hat\omega\approx\Delta\theta/t\). **Selected \(\hat\omega_L\): [TODO] rad/s; \(\hat\omega_R\): [TODO] rad/s.** Note whether clockwise and counterclockwise turns differ.

### Final calibration parameters

| Parameter | Final value | Units / interpretation |
|---|---|---|
| Forward motor command | [TODO] | normalized `[L,R]` |
| Left motor command | [TODO] | normalized `[L,R]` |
| Right motor command | [TODO] | normalized `[L,R]` |
| Forward speed \(\hat v_f\) | [TODO] | m/s |
| Left turn rate \(\hat\omega_L\) | [TODO] | positive rad/s |
| Right turn rate \(\hat\omega_R\) | [TODO] | negative rad/s |
| Effective startup time correction, if used | [TODO or not used] | s |

## 4. Waypoint algorithm and software

The planned implementation follows each requested **pose** using turn → drive → final turn:

1. Read `(x,y,theta)` rows from `config/waypoints.txt` using comma separation. Multiply x and y by `0.8`; preserve theta in radians.
2. Manually initialize the robot at the first pose.
3. For consecutive poses, compute \(\Delta x,\Delta y\), distance \(d=\sqrt{\Delta x^2+\Delta y^2}\), and desired travel bearing \(\phi=\operatorname{atan2}(\Delta y,\Delta x)\).
4. Rotate by \(\operatorname{wrap}(\phi-\theta_{\mathrm{previous}})\).
5. Drive distance \(d\) using the calibrated forward-command-to-duration conversion.
6. Rotate by \(\operatorname{wrap}(\theta_{\mathrm{target}}-\phi)\) to meet the **specified target heading**.
7. Update the **predicted** pose to the target. For zero-distance segments, skip the travel-bearing calculation and rotate directly to the target heading.

Angle wrapping: \(\operatorname{wrap}(\alpha)=(\alpha+\pi)\bmod2\pi-\pi\).

**Command transport:** Publish a `Float32MultiArray([L,R])` stream at 25 Hz. The provided `motor_control.py` subscribes to `/motor_commands`, clips values to `[-0.5,+0.5]`, and sends JSON commands over serial at ~20 Hz. Its safety watchdog zeros commands after a gap >0.15 s while the motor node is alive. Each motion primitive explicitly sends repeated zeros at completion.

**Actual code and adjustments:** [TODO: describe final implementations of `calibrate_motion.py`, `waypoint_controller.py`, any `setup.py` edits, any timing offsets, any changes to provided controller.]

## 5. Run results and video

**Video (unlisted YouTube URL):** [TODO]

**Complete run outcome:** [TODO: date/time, repetitions, final error, description of visible path].

## 6. Performance analysis and limitations

Replace these prompts with observations supported by your experiments:

- **Translation error:** [TODO: difference between predicted and measured translation, trends with duration].
- **Turning error:** [TODO: left/right asymmetry, repeated-turn drift, 90°/180° errors].
- **Repeatability:** [TODO: variation across repeated calibration trials].
- **Sources of uncertainty:** Wheel slip/skid steering, motor dead zone, startup response, floor friction, battery condition, command-timing variability, manual measurement error.
- **Long-route accumulation:** [TODO: how initial heading errors affected later position errors].
- **Design trade-off:** Simple turn-and-drive controller versus smooth continuous trajectory; explain why you selected this approach.
- **Potential improvements:** Camera/landmark feedback, wheel encoder data if available, refined command characterization, and better velocity/acceleration profiling. These were not required for the present open-loop demonstration.

## 7. How to run the code (reproducibility)

ROS 2 Jazzy and the official course `robot_control` package are required. The working tree should include `package.xml`, `setup.py`, its Python package sources, `config/`, and calibration files.

**Build once (on Rubik Pi):**

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select robot_control
source install/setup.bash
```

**Terminal 1 — hardware serial bridge:**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_control motor_control
```

**Terminal 2 — calibration script** (no new ROS entry point needed yet):

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/ros2_ws/src/rubikpi_ros2/robot_control/robot_control/calibrate_motion.py forward 0.5 --speed 0.2
python3 ~/ros2_ws/src/rubikpi_ros2/robot_control/robot_control/calibrate_motion.py left 0.2 --speed 0.5
```

**Terminal 2 — final autonomous navigation:** [TODO: exact command after implementing/registering waypoint controller, including file paths].

Ensure both terminals use the same ROS domain; run only one publisher that sends motion commands at a time. The robot must start in a known physical pose. Do not run unattended or near edges/obstacles.

## 8. Submission contents

Include `report.pdf` (exported from this document), this Markdown source, runnable `robot_control` ROS package source including `setup.py` and `package.xml`, `waypoint_controller.py`, `calibrate_motion.py`, `config/waypoints.txt`, `config/calibration.json`, and `calibration/trials.csv` as appropriate. Exclude generated `build/`, `install/`, `log/`, and Python caches. Link the **unlisted** YouTube demonstration in the PDF; the video itself does not need to be in the ZIP.
