# CSE 276A — Homework 1: Open-Loop Waypoint Navigation

**Student:** Rui Zhang
**Due:** October 16, 2026  
**Report status:** Draft — translation/motor-balance (30 trials) and rotation calibration (12 trials) recorded. Large-angle confirmation, full-route results, and video still pending.  
**Video:** [TODO: unlisted YouTube URL]

> **Submission:** This Markdown is the editable report source and the project README. Export a final `report.pdf` from it and include both the PDF and the runnable `robot_control` package in the ZIP.

## 1. Objective and experiment setup

The goal is to traverse the assigned sequence of planar poses \((x,y,\theta)\) autonomously using Python and ROS 2 on a four-wheel skid-steering robot. We approximate the vehicle as differential drive and use open-loop, time-based motor commands. 

- **Platform:** Wave Rover + Rubik Pi; Ubuntu 24.04, ROS 2 Jazzy.
- **Controller architecture:** `waypoint_controller.py` (to be implemented) publishes `std_msgs/msg/Float32MultiArray` messages on `/motor_commands`; the provided `motor_control.py` forwards them over USB serial (`/dev/ttyUSB0`) to the rover's motor controller.
- **Coordinate convention:** world +x right/forward at the starting pose, +y 90° counterclockwise, yaw positive CCW, yaw angles in radians.
- **Starting pose assumption:** manually place robot center at `(0,0)`, yaw `0` before every full run.
- **Physical scale:** 1 coordinate unit in the provided file corresponds to **0.8 m**, following instructor's permission for a scaled workspace. 
- **Test surface:** dorm low-pile carpet (used in all reported calibration trials).
- **Usable test area:** about 1.6 m × 1.6 m, with robot clearance already considered.
- **Measurement method:** Forward experiments: Straight tape measure laid alongside the robot and a box (kept below cable height) used as a repeatable reference marker. A set square was used to measure the rightward offset after motion. For each trial, the longer recorded “straight” value was the hypotenuse from start to end, and the recorded lateral value was the perpendicular leg, signed according to whether the endpoint was left or right of the initial forward axis. The unrecorded forward-axis leg and displacement bearing were computed using right-triangle geometry. The final orientation of the robot body was not independently measured in these forward trials. Photos of the measurement arrangement were recorded.

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

- Forward command **provisional** `(+0.130, +0.200)`: \(x\approx \hat v_f t\), where \(x\) is displacement along the initial forward axis and \(\hat v_f\) is only an empirical mean rate for the **tested 4.0 s command**. A 4.0 s command produced 0.79797 m mean forward displacement (2 trials); this is not yet a speed model validated over multiple durations.
- Left in-place turn `(-0.500, +0.500)`: signed yaw \(\Delta\theta_L>0\); magnitude characterized below.
- Right in-place turn `(+0.500, -0.500)`: signed yaw \(\Delta\theta_R<0\); magnitude characterized below.

These approximations assume reasonably repeatable motor response, floor friction, battery condition, and motion intervals. Short motions may show startup delay or acceleration effects; if measurements support it, use an empirical intercept or a duration lookup rather than assuming perfect linearity.

**Important open-loop limitation:** Predicted pose is updated using the requested displacement and rotation, **not** an observed pose. Slip and timing error therefore accumulate across the path.

## 3. Calibration method and results — 4-point rubric category

**Procedure:** Mark a reference origin, axis, and headings on the floor. Start each trial from the same pose. Send a motor command at 25 Hz for a specified interval, then publish repeated zero commands. After the robot stops, measure actual displacement and/or angular change. Store all trials in `calibration/trials.csv` (including command values, duration, measured result, and notes).
### Translation calibration — equal-command baseline (15 trials)

**Fixed variables:** motor command `[+0.200,+0.200]`, same low-pile carpet and measurement arrangement. **Varied variable:** command duration (1.0, 1.5, and 3.0 s), five independent starts per duration. Full raw measurements and derived values are in `calibration/trials.csv`.

**Measurement geometry and conventions:** Let `h` be the measured hypotenuse from the start reference to the endpoint and `r` the measured nonnegative lateral offset. Set signed `y = -r` for right deviation and `y = +r` for left deviation. The missing forward-axis component and net displacement bearing are:

\[
 x=\sqrt{h^2-y^2},\qquad \alpha=\operatorname{atan2}(y,x).
\]

For *all* `motion=forward` rows, `distance_m` contains the **forward-axis component `x`**, while `angle_rad` contains the **net displacement bearing `α`**, *not* the robot's final yaw. The hypotenuse `h` is the start-to-end displacement magnitude, not the integrated distance traveled along a curved trajectory. The original hypotenuse and signed lateral measurements remain in the CSV `notes`.

| Command `[L,R]` | Duration | Trials | Mean forward `x` (m) | Sample SD `x` (m) | Mean signed lateral `y` (cm) | Mean displacement bearing | Mean `x/t` (m/s) |
|---|---:|---:|---:|---:|---:|---:|---:|
| `[0.200,0.200]` | 1.0 s | 5 | 0.24654 | 0.00808 | −1.12 | −2.60° | 0.24654 |
| `[0.200,0.200]` | 1.5 s | 5 | 0.38504 | 0.00992 | −2.08 | −3.10° | 0.25669 |
| `[0.200,0.200]` | 3.0 s | 5 | 0.78429 | 0.00950 | −7.82 | −5.69° | 0.26143 |

The increasing measured rightward offset is consistent with systematic rightward curvature. The final *body* heading was not directly measured, so the displacement bearing cannot be used as a yaw measurement.

**Initial timing model for the baseline command**, estimated using linear least squares over the 15 trials (1.0–3.0 s):

\[
 \hat x_{[0.2,0.2]}(t)=0.26825t-0.01983\quad \text{m},\qquad 1\leq t\leq 3\ \text{s}.
\]

The intercept is an empirical correction, not proof of a specific physical dead time. This baseline timing model **must not be reused** for the corrected left/right command pair.

### Straightness / left-command sweep — completed (15 additional trials)

**What varied:** Left normalized motor command `L` in the range `0.080`–`0.180`. **What stayed fixed:** Right command `R=+0.200`, carpet, measurement procedure; each of the 3.0 s and 4.0 s subsets used a fixed duration. A separate 6.0 s test was conducted for `L=0.100`. The initial survey used one trial per setting; the final 4.0 s candidates were repeated. Consequently, the observations are strongest for the repeated 4.0 s settings, and the single-trial survey is exploratory.

**Raw trial-level measurements:** 15 additional rows `F4.01`–`F4.15` have been appended to `calibration/trials.csv`. Signs in the table below follow the coordinate convention: negative is rightward; positive is leftward.

| Left `L` | Right `R` | Duration | n | Mean forward `x` (cm) | Mean signed lateral `y` (cm) | Mean displacement bearing (deg) |
|---:|---:|---:|---:|---:|---:|---:|
| 0.180 | 0.200 | 3.0 s | 1 | 74.27 | −5.80 | −4.47 |
| 0.160 | 0.200 | 3.0 s | 1 | 69.37 | −4.30 | −3.55 |
| 0.140 | 0.200 | 3.0 s | 1 | 64.68 | −4.00 | −3.54 |
| 0.120 | 0.200 | 3.0 s | 1 | 58.65 | −2.30 | −2.25 |
| 0.100 | 0.200 | 3.0 s | 1 | 49.88 | −1.30 | −1.49 |
| 0.095 | 0.200 | 3.0 s | 1 | 46.70 | +0.10 | +0.12 |
| 0.090 | 0.200 | 3.0 s | 1 | 45.20 | +0.20 | +0.25 |
| 0.080 | 0.200 | 3.0 s | 1 | 41.39 | +0.70 | +0.97 |
| 0.100 | 0.200 | 6.0 s | 1 | 98.43 | +5.80 | +3.37 |
| 0.140 | 0.200 | 4.0 s | 1 | 83.39 | −1.00 | −0.69 |
| 0.135 | 0.200 | 4.0 s | 3 | **80.56** | **−0.60** | **−0.43** |
| 0.130 | 0.200 | 4.0 s | 2 | **79.80** | **−0.65** | **−0.47** |

**Interpretation and selection:** Reducing the left command substantially reduced rightward drift, but also lowered forward travel speed. In the 3.0 s exploratory subset, an offset sign change occurred between `L=0.100` (1.3 cm right) and `L=0.095` (0.1 cm left); however, those commands covered only ~0.47–0.50 m. **Do not conclude that `L=0.095` is best for an 0.8 m segment.** The 6.0 s test at `L=0.100` ended 5.8 cm to the *left*, illustrating that lateral error also depends on commanded duration and/or distance.

For the target **0.8 m waypoint spacing**, the 4.0 s experiments are more directly relevant:

- `[+0.135,+0.200]` (n=3): 0.80563 m mean forward-axis displacement, 0.60 cm mean **rightward** offset; individual offsets ranged from 0.1 cm left to 1.1 cm right.
- `[+0.130,+0.200]` (n=2): 0.79797 m mean forward-axis displacement, 0.65 cm mean **rightward** offset; individual offsets were 0.4 and 0.9 cm right.

**Provisional choice for 0.8 m forward legs:** `[L,R]=[+0.130,+0.200]` for **4.0 s**, because the observed mean forward displacement is within ~0.2 cm of 0.8 m. The `[+0.135,+0.200]` setting is also defensible and has an extra repeat; the two configurations have essentially indistinguishable mean lateral offsets at this measurement precision. The 4-second effective mean rate for the provisional choice is approximately **0.19949 m/s**; this is **not yet validated** as a constant-speed model for other durations or for 1.131 m diagonal segments. Confirming longer forward movements or the timing relationship remains a separate task.

**Comparison on approximately 0.8 m moves:** the original equal-command 3.0 s baseline gave 0.78429 m forward displacement and 7.82 cm rightward offset (five trials); the selected 4.0 s correction gave 0.79797 m and 0.65 cm rightward offset (two trials). This is a useful demonstration of reduced lateral offset over similar distances, **not** a same-duration comparison or proof of a universally superior command ratio.

**Limitations:** Final body yaw was not independently measured. Apparent lateral offsets may reflect curvature, initial alignment, floor effects, and measurement uncertainty. The largest tested long run (6 s) is a single observation. Further full-route evaluation is required before claiming robust accuracy.

### Rotation calibration — short-duration sweep (12 trials)

**Procedure:** Starting from a fixed marked orientation on the same low-pile carpet, command an in-place left rotation with `[L,R]=[-0.500,+0.500]`, or right rotation with `[+0.500,-0.500]`. Keep command values and publishing rate fixed; vary command duration among 0.2, 0.4 and 0.6 s, with two independent resets/trials per direction and duration. Measure the **actual final body rotation** in degrees using the physical heading reference. Unlike translation trials, this is directly measured body yaw, not the bearing of start-to-end displacement. Center displacement was not measured, so `distance_m` is blank rather than falsely set to zero. Right-turn angles are stored as negative radians in `calibration/trials.csv`.

| Turn | Input `[L,R]` | Duration | Trial rotations (deg, magnitude) | Mean signed rotation | Sample SD (deg) |
|---|---|---:|---|---:|---:|
| Left (CCW) | `[-0.500,+0.500]` | 0.2 s | +22, +16 | +19.0° | 4.24 |
| Left (CCW) | `[-0.500,+0.500]` | 0.4 s | +33, +43 | +38.0° | 7.07 |
| Left (CCW) | `[-0.500,+0.500]` | 0.6 s | +50, +56 | +53.0° | 4.24 |
| Right (CW) | `[+0.500,-0.500]` | 0.2 s | −22, −23 | −22.5° | 0.71 |
| Right (CW) | `[+0.500,-0.500]` | 0.4 s | −48, −37 | −42.5° | 7.78 |
| Right (CW) | `[+0.500,-0.500]` | 0.6 s | −61, −54 | −57.5° | 4.95 |

**Estimated angular motion model (exploratory):** Fit ordinary least-squares lines using all six measured magnitudes per direction; positive/negative signed yaw is restored afterward:

\[
 \hat\theta_L(t) \approx (+85.00 t + 2.67)^\circ,
 \qquad
 \hat\theta_R(t) \approx -(87.50 t + 5.83)^\circ,
 \qquad 0.2\le t\le0.6\;\mathrm{s}.
\]

The fitted slopes are about **+1.484 rad/s** (left) and **−1.527 rad/s** (right). The nonzero intercepts reflect the *empirical line fitting the sampled intervals*, **not a physically meaningful instantaneous jump in orientation**. An alternative through-origin least-squares fit gives mean rates **+1.583 rad/s** (90.71°/s) and **−1.745 rad/s** (100.00°/s), respectively. The fitted rates and predicted durations are not ground-truth wheel velocities. We should **not assume either model remains accurate for 1–2 seconds** until confirmed with new measurements, especially because the sample size is small and the repeated 0.4-second trials varied by 10–11°.

**Initial extrapolated commands for confirmation only:** From the affine fit, a nominal 45° turn would take approximately **0.498 s left / 0.448 s right**; 90° approximately **1.027 s left / 0.962 s right**; 180° approximately **2.086 s left / 1.990 s right**. These are **untested predictions**, not verified calibration settings. Start with the 90° left/right confirmation trials and adjust based on *actual measured yaw*, then validate the necessary 180° maneuver. The 0.2–0.6 s short-pulse results support modeling but do not establish full-size rotation accuracy.

**Interpretation:** Both direction and duration affect the observed turning response. Under the same magnitude `0.5`, clockwise trials tended to turn further than counterclockwise trials; e.g., average 0.6 s rotations were **57.5° right versus 53.0° left**. Trial-to-trial variability was noticeable, indicating the need for realistic error bounds in the final open-loop route. The motor output is a normalized command, not a directly observed wheel-speed measurement. The ideal differential-drive model motivates in-place turning with opposite-sign side commands, while calibration supplies the actual command-duration conversion on the carpet.

**Pending:** [TODO: measured 90° confirmation, measured 180° confirmation, any retuned durations. Update the controller parameters only after these tests.] 

### Final calibration parameters

| Parameter | Final value | Units / interpretation |
|---|---|---|
| Forward motor command | **Provisional `[+0.130,+0.200]`** | normalized `[L,R]`; selected for 0.8 m / 4.0 s |
| Left motor command | **Provisional `[-0.500,+0.500]`** | normalized `[L,R]`; used for measured calibration trials |
| Right motor command | **Provisional `[+0.500,-0.500]`** | normalized `[L,R]`; used for measured calibration trials |
| Forward speed \(\hat v_f\) | **Provisional 0.19949** | m/s; mean forward-axis displacement / 4 s, based on 2 repetitions; not yet a multitime fit |
| Left turn rate \(\hat\omega_L\) | **Provisional +1.484** | rad/s; affine-fit slope over 0.2–0.6 s; confirm longer turns |
| Right turn rate \(\hat\omega_R\) | **Provisional −1.527** | rad/s; affine-fit slope over 0.2–0.6 s; confirm longer turns |
| Effective startup time correction, if used | **Not yet estimated for corrected command** | s; old 0.2/0.2 model should not be transferred |

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
- **Turning error and repeatability:** For identical 0.6-second magnitude-0.5 commands, left turns averaged 53.0° (n=2), and right turns averaged 57.5° in magnitude (n=2). At 0.4 s, repeated trials differed by 10° left and 11° right. These are directly measured yaw changes; final 90°/180° accuracy and accumulated route-heading error remain [TODO].
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
