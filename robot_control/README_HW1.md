# CSE 276A — Homework 1: Open-Loop Waypoint Navigation

**Student:** Rui Zhang
**Due:** October 16, 2026  
**Report status:** Physical experiments completed and documented: **69 standalone motion-calibration trials**, **8 measured action-by-action route checkpoints**, an initial six-leg open-loop run with substantial position drift, and a later attempted run with provisional turn-center compensation that **did not reliably track the final waypoints**. The numerical endpoint error of the full route was not measured.  
**Video (unlisted YouTube):** [https://youtu.be/eSfmiSPTKbU?feature=shared](https://youtu.be/eSfmiSPTKbU?feature=shared)

**Final update (October 16):** After enabling provisional turn-center compensation and revising the forward speed, a further physical attempt still accumulated substantial heading drift. By the `(2,2)` waypoint, the robot's orientation was incorrect enough that its next movement headed approximately toward `(1,2)` rather than the required `(1,1)`. This is a qualitative observation, not a surveyed final pose. The compensated controller therefore **did not demonstrate reliable complete-route waypoint tracking**. The experiment supports the limitations discussion below. The 69 standalone calibration trials and eight action-by-action checkpoints remain the quantitative evidence; this additional attempt is recorded qualitatively. **Unlisted demonstration video:** [https://youtu.be/eSfmiSPTKbU?feature=shared](https://youtu.be/eSfmiSPTKbU?feature=shared).

> **Submission note:** The assignment asks for a report describing the **kinematic model, waypoint/control algorithm, Python/ROS 2 implementation, calibration procedure, and observed performance**, plus the **unlisted YouTube video URL**. This Markdown is the editable report source. Export it to PDF for the course submission and include that PDF with the runnable ROS package in the ZIP; the video itself is not required inside the ZIP.

## 1. Objective and experiment setup

The goal is to traverse the assigned sequence of planar poses \((x,y,\theta)\) autonomously using Python and ROS 2 on a four-wheel skid-steering robot. We approximate the vehicle as differential drive and use open-loop, time-based motor commands. 

- **Platform:** Wave Rover + Rubik Pi; Ubuntu 24.04, ROS 2 Jazzy.
- **Controller architecture:** `waypoint_controller.py` publishes `std_msgs/msg/Float32MultiArray` messages on `/motor_commands`; the provided `motor_control.py` forwards them over USB serial (`/dev/ttyUSB0`) to the rover's motor controller.
- **Coordinate convention:** world +x right/forward at the starting pose, +y 90° counterclockwise, yaw positive CCW, yaw angles in radians.
- **Starting pose assumption:** manually place robot center at `(0,0)`, yaw `0` before every full run.
- **Physical scale:** 1 coordinate unit in the provided file corresponds to **0.8 m**, following instructor's permission for a scaled workspace. 
- **Test surface:** dorm low-pile carpet (used for the trials tabulated here). Later 8 turning trials were carried out after an improvised painter's-tape alteration around the problematic right-rear wheel; this is a different mechanical condition from the earlier calibration sessions.
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

## 3. Calibration method and results

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

**Historical predictions from the short-duration fit:** Before 90° tests, extrapolating the 0.2–0.6 s fit predicted a 90° turn at **1.027 s left / 0.962 s right** and 180° at **2.086 s left / 1.990 s right**. Those predictions have now been **superseded by the actual 0.95–1.00 s tests below**. The longer observed rotations were generally larger than the short-pulse model predicted; do not use the old 180° extrapolations.

**Interpretation:** Both direction and duration affect the observed turning response. Under the same magnitude `0.5`, clockwise trials tended to turn further than counterclockwise trials; e.g., average 0.6 s rotations were **57.5° right versus 53.0° left**. Trial-to-trial variability was noticeable, indicating the need for realistic error bounds in the final open-loop route. The motor output is a normalized command, not a directly observed wheel-speed measurement. The ideal differential-drive model motivates in-place turning with opposite-sign side commands, while calibration supplies the actual command-duration conversion on the carpet.

 

### Rotation calibration — 90° confirmation (12 additional trials)

**Controlled variables:** The same low-pile carpet, initial body orientation, ROS publishing frequency (25 Hz), and fixed command magnitude of 0.500 as in the previous rotation tests. **Varied variable:** turn duration, 0.95 s versus 1.00 s. Three independent trials were recorded per duration and turn direction, measuring actual body yaw in degrees, not displacement bearing. Positive angles mean counterclockwise (left) and negative angles mean clockwise (right). No wheel-center translation measurements were taken.

| Direction | Input `[L,R]` | Duration | Recorded rotations (deg) | Mean signed rotation | Sample SD (deg) | Mean magnitude error from 90° |
|---|---|---:|---|---:|---:|---:|
| Left (CCW) | `[-0.500,+0.500]` | 0.95 s | +92, +84, +88 | +88.00° | 4.00 | −2.00° |
| Left (CCW) | `[-0.500,+0.500]` | 1.00 s | +96, +107, +98 | +100.33° | 5.86 | +10.33° |
| Right (CW) | `[+0.500,-0.500]` | 0.95 s | −97, −101, −100 | −99.33° | 2.08 | +9.33° |
| Right (CW) | `[+0.500,-0.500]` | 1.00 s | −102, −113, −91 | −102.00° | 11.00 | +12.00° |

**Comparison with the earlier model:** Previously, the short-pulse affine model predicted approximately +83.4° left and −88.96° right at 0.95 s, and +87.67° left and −93.33° right at 1.00 s. Measured 0.95–1.00 s rotations are generally larger; the original model was not reliable for longer extrapolation.

**Updated empirical model (descriptive, not physical wheel-speed model):** Refit a least-squares line to **all 12 measurements per direction**, including the short-duration and 90° trials. Using rotation *magnitude* in degrees:

\[
 |\Delta\theta_L(t)| \approx (98.40\,t - 2.23)^\circ,\qquad
 |\Delta\theta_R(t)| \approx (101.60\,t + 0.90)^\circ.
\]

Restore `+` sign for left, `−` sign for right; `t` is in seconds. Nominal slopes are **+1.718 rad/s left** and **−1.773 rad/s right**. This model is only an approximate interpolation over **0.2–1.0 s**. Its nonzero intercepts are empirical artifacts, not actual instantaneous rotations at t=0. Most importantly, it **must not be extrapolated to 180° without verification**. The observed spread of the 1.00 s right turns was 22° (91° to 113°), substantially limiting precision.

**Provisional 90° timing choices:** 

- **Left (CCW):** 0.95 s yielded **+88.0° average** (three trials). Use **0.95 s** initially for a 90° left turn, with observed variability.
- **Right (CW):** 0.95 s yielded **−99.33° average**, an overshoot of 9.33°. A first unverified prediction for a 90° right turn from the updated full-range fit is **0.877 s**, rounded to **0.88 s** for confirmation. Until tested, do **not** label 0.88 s a calibrated value.
- **180° clockwise (required at waypoint 4), at this EARLIER calibration stage:** Not yet measured under the original conditions. A historical linear extrapolation would have predicted ~1.76 s. **Later**, the altered-condition trials directly measured 170° at 2.10 s and 182°/188° at 2.20 s, as recorded below.

**Interpretation for the report:** Turning response depends on duration and direction. At 0.95 s, left turns were close to 90° but the right turns overshot. Under identical settings, the measured yaw still varies. Because the navigation system is open-loop, even small heading errors propagate into the next translation and accumulate over the full six-segment path.

**Historical pending items (now partially addressed in a changed mechanical setup):** After these early trials, a hardware performance deterioration occurred. Later clockwise 90° and 180° measurements were recorded *after painter's tape was applied near the problematic right-rear wheel*, as documented below. They should not be pooled with the original hardware-condition trials as though identical.


### Subsequent drivetrain deterioration and repeatability check (7 additional left-turn trials)

**Change in conditions and diagnostic observations:** After a break between testing sessions, in-place turns on the same carpet became slow and jerky, often moving only a few degrees before partially springing back. The motor-command topic continued to publish consistently (measured approximately **24.971 Hz** with reported maximum gap **0.044 s**, comfortably below the course motor bridge's 0.15 s watchdog). The robot could still turn substantially better with all four wheels **touching a smooth tabletop**. A stretched bedsheet over the carpet only slightly improved turning; attempts at smoother forward-curving commands barely worked. The chassis battery appeared adequately charged, but voltage under turning load was not measured. The actual cause was **not identified**.

After this change, seven more **left-turn** tests were conducted at normalized inputs `[-0.50,+0.50]` on the carpet:

| Trial group | Duration | Measured angles (positive CCW) | Mean | Sample SD |
|---|---:|---|---:|---:|
| L90_new1–5 | 1.50 s | 87°, 83°, 86°, 89°, 86° | **86.2°** | **2.17°** |
| L180_new1 | 3.00 s | 175° | 175° | n=1 |
| L180_new2 | 3.05 s | 182° | 182° | n=1 |

Fitting an approximate **through-origin angle-magnitude/time relation** to these seven **post-deterioration** measurements gives

\[
|\Delta\theta_L|\approx(58.425^\circ/\mathrm{s})\,t,
\qquad \omega_L\approx +1.020\,\mathrm{rad/s}.
\]

Thus the unverified *planning estimates* are approximately **1.54 s for a 90° left turn** and **3.08 s for a 180° left turn**. The measurements at 1.50 and 3.00–3.05 s support this rate over these two ranges, but should not be interpreted as proof that the robot rotates around a fixed center or that the model remains valid after a hardware repair.

**Important hardware observation:** During these later left turns, the right-side wheels appeared to move disproportionately, producing substantial **leftward chassis translation**. The center displacement was not measured in those standalone turning tests; it was measured later in the action-by-action route experiment. In a subsequent attempt, the right-front wheel appeared to move while the **right-rear wheel barely rotated**. Testing was stopped and the robot was powered off to cool down. This is a suspected wheel-specific or load-sensitive drivetrain/power issue, **not a confirmed diagnosis**. A successful yaw measurement is therefore **not proof of accurate waypoint position**. Repeated high-load turns are inappropriate until this behavior is inspected and resolved.

**Non-comparability:** Earlier right-turn measurements (including the roughly 100° rotations at 0.95 s) were collected *before* the observed performance deterioration. They must not be presented as validated for the current hardware condition. At that stage there were not yet post-deterioration right-turn data; the eight subsequent painter-tape-condition turning trials recorded below **do** include right-turn measurements. Earlier straight-line performance was later tested qualitatively in waypoint segments 1–2, but accurate new distances were not supplied.

### Additional rotation calibration with painter's tape (8 new trials; mechanically altered condition)

**Change in experimental condition:** After observing that the right-rear wheel sometimes barely rotated while the right-front wheel rotated, painter's tape was applied to the problematic right-rear area. The precise mechanical arrangement and its effect on wheel freedom were not independently evaluated. This is **not evidence that the drivetrain was repaired**. The eight results below were obtained with magnitude-0.5 opposite-sign commands on the carpet in this altered configuration. The operator measured signed body yaw; the center translation was **not measured in those eight isolated turn trials**, but was subsequently estimated from full-route checkpoints. Commands are `[-0.500,+0.500]` for left and `[+0.500,-0.500]` for right.

| Direction | Command duration | Recorded magnitudes (degrees) | Mean | Sample SD |
|---|---:|---|---:|---:|
| Left | 0.70 s | 93, 86 | **89.5°** | 4.95° |
| Left | 1.40 s | 182 | 182.0° | n=1 |
| Right | 0.90 s | 86, 87 | **86.5°** | 0.71° |
| Right | 2.10 s | 170 | 170.0° | n=1 |
| Right | 2.20 s | 182, 188 | **185.0°** | 4.24° |

**Left-turn timing model:** Through-origin least squares across these three left-turn observations estimates a rate of **129.29°/s** (2.257 rad/s), giving `t ≈ angle_degrees / 129.29`. This is consistent with 89.5° at 0.70 s and 182° at 1.40 s within the small sample's uncertainty. The empirical angles do not establish that the rover's center remained at the commanded pose.

**Right-turn timing is nonlinear over the tested durations:** The two 0.90 s clockwise turns averaged 86.5° (about **96.11°/s**), whereas the 2.20 s clockwise turns averaged 185° (about **84.09°/s**), and 2.10 s yielded 170°. Fitting a *single* through-origin rate to all five current-condition right turns gives **84.45°/s**, but that rate would predict ~1.07 s for 90°, notably longer than the directly observed ~0.90 s for 86.5°. A single rate does not explain both conditions well. This could reflect time-dependent slip, changing contact mechanics, load, or measurement scatter; the data do not identify a unique cause.

**Updated control choice:** For route planning, use a *piecewise linear duration calibration* in `config/calibration.json` rather than one angular-rate constant for all angles. The anchors are measured mean **angle magnitudes versus motor-on times**:

- Left: `(0°,0 s)`, `(89.5°,0.70 s)`, `(182°,1.40 s)`.
- Right: `(0°,0 s)`, `(86.5°,0.90 s)`, `(185°,2.20 s)`.

Time is linearly interpolated between adjacent anchors. For example, a desired **90° right turn** gives ~**0.946 s**, while a **180° right turn** gives ~**2.134 s**. The zero-origin is an assumed interpolation anchor; **45° and 135° commands have not been independently measured under the taped condition**. Avoid extrapolating above the largest measured angle. The updated `waypoint_controller.py` reads `duration_curve_deg_sec`; the `degrees_per_second` fields remain as backward-compatible fallback rates only. This is *calibration for the observed altered state*, not proof of safe or repeatable operation under load.

**Safety limitation:** If the painter's tape wraps around, holds stationary, or contacts any rotating wheel/axle/gear, powered use could increase stalling, overheating, or drivetrain damage. Stop operation, remove the unsafe restraint, and have the problem inspected. A reliable physical waypoint demonstration requires that the drivetrain function without a binding wheel. We do not infer that the tape is a safe repair from these angles.

**CSV convention:** These eight records use `L90_tape1`–`R180_tape3`. Their `angle_rad` fields record measured signed *body yaw*, and `distance_m` remains blank because lateral/center displacement was not quantified. The painter-tape mechanical condition is labeled in each notes field so these trials are not mixed with the previous sessions.

### Controller calibration parameters (provisional; mechanically altered condition)

| Parameter | Current value | Meaning / limitation |
|---|---|---|
| Scale | `0.8 m/coordinate unit` | Applied only to x and y |
| Forward commands | `[+0.130,+0.200]` | Pre-deterioration tests: mean 0.79797 m in 4.0 s (n=2); not reverified after the tape alteration |
| Forward speed | `0.174414 m/s` | Provisional fitted from four shorter-than-commanded in-route movements; earlier isolated estimate was 0.19949 m/s |
| Left commands | `[-0.500,+0.500]` | New altered-condition turns |
| Left fallback angular rate | `129.28571°/s` | Through-origin fit to 3 current-condition trials; piecewise duration anchors preferred |
| Right commands | `[+0.500,-0.500]` | New altered-condition turns |
| Right fallback angular rate | `84.44940°/s` | Through-origin fit to 5 current-condition trials; **poor at 90°** |
| Left duration anchors | `[0°,0 s], [89.5°,0.70 s], [182°,1.40 s]` | Time interpolation in updated controller |
| Right duration anchors | `[0°,0 s], [86.5°,0.90 s], [185°,2.20 s]` | Time interpolation in updated controller |
| ROS command rate | `25 Hz` | Motor bridge has a 0.15 s no-message watchdog |
| Inter-motion settle | `0.30 s` | Program setting, not an observed physical stopping time |
| Hardware verification | **Not verified safe** | Painter's tape does not establish wheel reliability; inspect before powered use |

**Compatibility:** `calibration.json` now contains optional `duration_curve_deg_sec` entries. To use those interpolated times, copy the corresponding **updated `waypoint_controller.py`**. The original waypoint script will ignore these anchors and use only `degrees_per_second`, which is inadequate for the right-turn response at both 90° and 180°. The forward-calibration parameters remain based on older experiments and are not revalidated in the taped condition.

### Initial physical waypoint tests (qualitative, current altered drivetrain configuration)

After implementing `waypoint_controller.py` and checking the six-leg dry-run on the Rubik Pi, two physical tests were reported successful:

| Test | Commanded trajectory | Predicted end pose | Observation |
|---|---|---|---|
| RT1 | Waypoint 0 → 1 | `(0.8 m, 0.0 m, 0 rad)` | Completed; no numerical endpoint measurement provided. |
| RT2 | Waypoint 0 → 1 → 2 | `(0.8 m, 0.8 m, 1.57 rad)` | Completed; during the left turn, a visually estimated **5–15 cm** unintended chassis displacement toward **negative world y** was observed. Precise endpoint pose was not recorded. |

This 5–15 cm is a rough **turn-induced translation observation**, not a measured endpoint position error or calibrated per-turn correction. It should not be subtracted automatically from waypoint coordinates. Because the robot's motion is skid steering, an asymmetric left/right drivetrain or unequal tire-ground interaction may cause the rotation center to translate. The current simple turn-and-drive model assumes in-place rotation and therefore neglects this shift. Such error will compound across the route, and an arbitrary fixed y offset would not apply equally to every subsequent robot orientation. The first two tests are logged separately in `calibration/route_tests.csv`; no extra timed trial rows were added to the 69-row `calibration/trials.csv`.

**Right-turn data status correction:** In the *current painter-tape configuration*, right turns **were** physically measured: 86° and 87° at 0.90 s, 170° at 2.10 s, and 182°/188° at 2.20 s. Thus the right-turn **timing** is experimentally supported in this configuration. This does not verify that the right-rear wheel is mechanically safe. In `config/calibration.json`, `timing_tested_current_configuration=true` records calibration evidence, whereas `verified_current_hardware=false` preserves the unresolved physical-safety warning. The controller uses timing-test status rather than physical-safety status to decide whether a separate *unverified timing* override is needed. Physical execution always requires explicit `--hardware-checked` acknowledgement by the operator.

### Full six-segment run — qualitative evaluation and next experiment

A first **complete six-leg physical run** was executed. The overall sequence of route directions was recognizable, but the observed position drift accumulated, and the first three legs appeared to steer somewhat **too far left**. On the segment nominally from file coordinate `(2,1)` to `(2,2)`, the robot was visually estimated near **`(2,1.7)` at the end of that forward movement, BEFORE the final heading correction**. With the 0.8 m/unit scale, this corresponds to a rough y shortfall of `0.3 × 0.8 = 0.24 m` relative to `(2,2)`. This is an approximate observation, **not** a surveyed endpoint position or evidence that the 0.8 m straight command traveled only 0.56 m: previous turn translation, biased heading, and actual forward distance may all contribute. After the following turn, position error appeared larger. Final full-run endpoint position was not measured. Entries are kept in `calibration/route_tests.csv`; **no invented numerical calibration rows** were added to `trials.csv` (69 calibration trials).

**Immediate diagnostic:** Run `--segments 4 --pause-after-actions` with fixed floor coordinate markers, stopping after each primitive. At the start of leg 4, note the robot center and yaw; after its initial left rotation, note the chassis center and yaw; after the following forward movement, note its position **before** the final right turn. This separates the 90°-turn translation, the forward segment's physical length, and the large final rotation's translation. Do not reposition the robot mid-run if treating the sequence as one open-loop trial.

**Targeted new measurement (two trials each, if hardware safe):** At initial yaw 0°, measure the chassis center before and after `left 0.70s --speed 0.5` and `right 0.90s --speed 0.5`. Record both signed displacement components in the initial **body frame**: `dx` positive forward, `dy` positive left. Also record the actual rotation angle. Put these observations in `calibration/turn_shift_measurements.csv`. We currently only have the broad **5–15 cm approximate world −y observation**, which is **insufficient to estimate both body-frame components**. No numeric compensator is enabled yet.

### Action-by-action route measurements and updated empirical model (latest run)

The rover was allowed to execute its prior uncompensated motion plan, with its center position measured externally in centimeters against fixed floor axes after selected individual actions. The new data are in `calibration/route_checkpoints.csv` and `calibration/turn_shift_measurements.csv`. **These were not 8 extra standalone calibration trials**: the separate `trials.csv` remains at 69 trials. The checkpoints below cover only the sequence through the **final right turn at original waypoint `(2,2)`** (index 4, zero-based). The user used conversational waypoint numbering; the action sequence is disambiguated using commanded movement types.

| Checkpoint | Center (m) | Actual body yaw | Difference since last checkpoint |
|---|---|---:|---|
| Forward 1 completed | (0.75, 0.00) | −3° | first 0.8 m leg (0.75 m actual) |
| Left turn 1 completed | (0.71, −0.11) | +83° | (−0.04, −0.11) m; yaw +86° |
| Forward 2 completed | (0.87, 0.61) | +78° | (+0.16, +0.72) m |
| Right turn 1 completed | (0.91, 0.65) | +8° | (+0.04, +0.04) m; yaw −70° |
| Forward 3 completed | (1.57, 0.72) | +9° | (+0.66, +0.07) m |
| Left turn 2 completed | (1.55, 0.66) | +85° | (−0.02, −0.06) m; yaw +76° |
| Forward 4 completed | (1.58, 1.31) | +86° | (+0.03, +0.65) m |
| Right ~180° turn completed | (1.71, 1.30) | −88° | (+0.13, −0.01) m; yaw −174° |

The nominal final pose of this four-leg sequence is `(1.60 m, 1.60 m, −90°)`. The measured final checkpoint after the fourth leg's final right turn was approximately `(1.71, 1.30, −88°)`, **11 cm beyond x and 30 cm short of y**, corresponding to approximately **31.95 cm position error**. The final full six-leg destination `(0,0,0)` was **not** measured in this diagnostic sequence.

**Forward motion is also undershooting.** Expressing each measured forward center displacement in the pre-action body frame, the four commanded 0.8 m legs produced forward components **75.0, 73.4, 66.3, and 65.0 cm**. Their mean is **69.94 cm**, giving a provisional effective speed of **0.17441 m/s** for the 4.01-second motions in this run, versus **0.19949 m/s** from the earlier independent straightness calibration. Since the forward response weakened throughout the sequence, a constant-speed correction is an approximation and may overshoot earlier legs. Forward distances beyond 0.8 m are still extrapolated.

**Turn-center displacement is directly identifiable.** For each measured turn, calculate its signed world displacement `d_w = p_after − p_before` and rotate into the pre-turn body frame: `d_b = R(−θ_before) d_w`. The observed turn body-frame shifts were:

| Turn | Actual yaw change | Body-frame forward shift | Body-frame left shift |
|---|---:|---:|---:|
| First left | +86° | −3.42 cm | −11.19 cm |
| First right | −70° | +4.74 cm | −3.08 cm |
| Second left | +76° | −2.91 cm | −5.61 cm |
| Right ~180° | −174° | −0.09 cm | −13.04 cm |

A **separate least-squares effective pivot** `c=(c_x,c_y)` was fitted for each turning direction using all two available measurements for that direction and the model `d_b(δ)=(I−R(δ))c`. This gives **left `c=(+0.0332,-0.0619) m`**, **right `c=(-0.0025,-0.0612) m`**. Equivalent *model-predicted*, not directly measured, shifts for a ±90° turn are **left `(-0.0287,-0.0952) m`** and **right `(+0.0587,-0.0637) m`**, in the pre-turn body frame. These values are now in `config/calibration.json` with `turn_center_shift.enabled=true`. This accounts for modeled translational shifts in the initial/final rotations by changing the subsequent planned bearing and distance; it does **not** measure the pose during autonomous operation or guarantee positional accuracy.

**Limitations:** Only two turns per direction were measured, and actual yaw sometimes fell considerably short of commanded yaw (for example −70° for a nominal right 90°). The robot's drivetrain condition has been unstable and the taped right-rear-wheel modification has not been verified safe. A constant effective pivot and forward speed are both approximations. Avoid claiming that the compensated dry-run predictions are actual measurements. Follow the new `--dry-run` intermediate-center printout and check room clearance before physical testing; even the nominal route touches x/y boundary values 0 and 1.6 m.

**Next verification:** On a mechanically safe rover, test only the first two legs at a pause after each action; measure body center and yaw. Check whether the compensated plan reduces the observed turn shift error at waypoint `(1,1)`. Do not repeat a full high-load route if any wheel is still binding.

### Final compensated-route attempt and outcome

After measuring the center after individual actions, I introduced an empirical effective-pivot model for left/right rotations and adjusted the estimated forward speed using the four measured forward legs. The controller recomputed planned turn bearings and drive lengths to compensate for predicted translation during turns. These changes were checked by software dry-run, but the physical run remained inaccurate.

In the subsequent physical attempt with **provisional turn-center compensation enabled** and a lower forward-speed estimate, heading errors continued to accumulate. At the nominal waypoint `(2,2)` (physical target `(1.6,1.6)` m), the rover was sufficiently misoriented that the following travel headed broadly toward `(1,2)` instead of the assigned next waypoint `(1,1)`. The required next displacement is `(-0.8,-0.8)` m (southwest, bearing `-135°`); the reported movement was more nearly west. This is a **qualitative direction comparison**, not a measured 45° heading error or final-pose measurement. The compensated model therefore was **not demonstrated to improve reliable whole-route tracking**, even though the geometry and ROS commands still ran. We do not claim that the final waypoint was reached or that the recorded video demonstrates accurate final convergence.

**Diagnosis:** The pivot-displacement model adjusts the *predicted position* after turns, but it still executes a planned yaw for a fixed duration. It cannot measure or repair the actual yaw after an under- or over-rotation. A variable heading error rotates the next meter-scale translation in the wrong direction, and any later compensation computed from the predicted pose can compound the error. In addition, the four measured 0.8 m forward actions varied from 0.65 to 0.75 m, indicating a constant forward-rate estimate is not reliable across the route. Wheel-ground interaction and the intermittently ineffective right-rear drive are plausible contributors; the observations do not isolate a single physical cause. A robust solution would require a sound drivetrain and pose/heading feedback, not only another open-loop duration adjustment.

**Interpretation:** Correcting translation during a turn does not correct the actual heading when turn angles vary. Even a small heading error changes subsequent drive direction, and a large error at a diagonal leg can make the path visibly wrong. The fitted pivot assumes repeatable wheel-ground interaction, but the robot showed variable turning response, a suspected right-rear-wheel fault, and different behavior on carpet versus a hard surface. Thus the model's prediction is not reliable enough to eliminate drift without sensing. The modified configuration was a provisional experimental workaround, not a verified drivetrain repair.

**Outcome:** The robot executed the full sequence of motion commands in the earlier uncompensated run, broadly following the path but with significant drift; the later compensated attempt **did not accurately traverse the complete waypoint sequence**. The most defensible next improvement would be periodic external heading/pose measurements or onboard localization and feedback, plus inspection of the drivetrain, rather than further open-loop corrections based on one run.

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

### Optional rotation-center translation model (added after full run)

The original model assumes that a turn changes yaw only, with no center translation. This is contradicted by the observed left-turn shift. For a simple approximate improvement, let the robot rotate through angle `δ` about an **effective fixed pivot** `c = (c_x,c_y)` in the body frame. Its center displacement in that frame is

\[
\begin{bmatrix}\Delta x_b\\\Delta y_b\end{bmatrix}
= (I-R(\delta))\begin{bmatrix}c_x\\c_y\end{bmatrix},
\qquad R(\delta)=\begin{bmatrix}\cos\delta&-\sin\delta\\\sin\delta&\cos\delta\end{bmatrix}.
\]

For each direction separately, we can infer `c` from a **direct measurement of both signed x and y center displacement during a 90° turn**. The implementation then predicts translation at other turn angles using this pivot approximation, rotates the local displacement into the world frame using the **pre-turn** yaw, and solves for a drive heading and distance that compensate for the predicted displacement of both the initial and final turns. This changes the **planned commands**, not the measured robot pose; the controller still has no localization feedback.

The assumption of a fixed effective pivot was fitted only to four in-route turns, so its predictions remain **unvalidated for other conditions and repeat runs**, including many 45°, 135°, or 180° turns, especially because the right-rear-wheel fault and carpet slip cause variable turning behavior. Small calculated correction turns may be below the reliable physical control resolution. Do not claim centimeter accuracy on the basis of modeled predictions alone.

**Calibration JSON interface (now populated from observed action checkpoints):** `turn_center_shift.enabled=true`; `left_90deg_body_m` and `right_90deg_body_m` contain **equivalent ±90° shifts fitted from measured full-run actions** (not direct 90° measurements). The controller uses these to predict turn-center movement and solve compensated bearings and distances. Setting `enabled=false` restores the original uncompensated plan. The model is approximate and has not yet been physically validated.

**Command transport:** Publish a `Float32MultiArray([L,R])` stream at 25 Hz. The provided `motor_control.py` subscribes to `/motor_commands`, clips values to `[-0.5,+0.5]`, and sends JSON commands over serial at ~20 Hz. Its safety watchdog zeros commands after a gap >0.15 s while the motor node is alive. Each motion primitive explicitly sends repeated zeros at completion.

**Implementation:** `calibrate_motion.py` sends supervised 25 Hz timing trials. `waypoint_controller.py` is a separate open-loop Python program that loads a comma-separated waypoint file and `config/calibration.json`, scales positions, computes geometric travel bearings, wraps angular differences to `[-pi, pi)`, and produces each `turn → drive → turn` command. It prints a **dry-run by default** (which works on a Mac without ROS). Physical motion requires `--execute` and a specific hardware-check acknowledgement. Right-turn execution is blocked only if the timing was not tested in the current configuration; the separate hardware-safe flag remains false. Optional `--pause-at-waypoints` stops after each waypoint. Added `--pause-after-actions` stops after **each turn or forward movement**, permitting direct measurements of turn-induced displacement and forward-motion undershoot (it does not itself correct pose). Once executing on Ubuntu, the script checks for the motor-control subscriber, sends continuous 25 Hz motor commands using a monotonic clock, sends repeated zeros between motions and at shutdown, and stops after a selected number of waypoint legs if `--segments N` is supplied. The existing `motor_control.py` is not modified. The revised controller interpolates empirically measured angle-duration anchors when `duration_curve_deg_sec` is present, and uses constant angular speed only as a fallback. `setup.py` can optionally register `waypoint_controller = robot_control.waypoint_controller:main`, but the code can also be run directly with `python3`. The complete **uncompensated** six-segment motion sequence has now been physically executed once with substantial qualitative position drift; the new compensation logic is software-tested only and not yet validated on the rover.

## 5. Run results and video

**Video (unlisted YouTube URL):** [https://youtu.be/eSfmiSPTKbU?feature=shared](https://youtu.be/eSfmiSPTKbU?feature=shared)

**Segmented test outcome:** `--segments 1` and `--segments 2` were completed on the robot, with qualitative turn-induced lateral shift of ~5–15 cm toward negative y during the left turn. Final endpoint position and heading were not numerically measured. See `calibration/route_tests.csv`.

**Complete run outcome:** An initial six-leg open-loop attempt executed the sequence of actions and followed the broad path pattern, but with substantial accumulated positional drift. The first three legs appeared left-biased. During that run, the robot was visually near `(2,1.7)` instead of `(2,2)` after the fourth forward movement (about **0.24 m short in world y**, rough visual estimate). In a **separate measured, action-by-action run** through waypoint `(2,2)`, its center after the final turn was `(1.71,1.30)` m with yaw `-88°`, versus the target `(1.60,1.60)` m with yaw about `-90°`: the measured checkpoint position error was `sqrt(0.11² + 0.30²) ≈ 0.320 m`. This is a waypoint-4 checkpoint error, **not a measured endpoint error for the entire route**. In a later compensated attempt, accumulated heading error caused the next movement from `(2,2)` to head broadly west toward `(1,2)` instead of southwest toward `(1,1)`. The latest compensated route did not accurately track the remaining waypoints. No numerical final-route endpoint was recorded.

## 6. Performance analysis and limitations

The following observations summarize measured and qualitative outcomes; numerical final-pose error was not measured:

- **Translation error:** Earlier independent forward calibration reached ~0.8 m at 4.0 s with `[0.130,0.200]`. During the full route, the approach to waypoint 4 appeared ~0.24 m short in the y direction **relative to nominal coordinates**, but this cannot be attributed to forward speed alone because it includes earlier rotations and heading errors. Measure the fourth leg's start, post-turn center, and post-drive center separately.
- **Turning error and repeatability:** In the original hardware condition, 0.95 s left turns averaged 88.0° (sample SD 4.0°) and right turns averaged 99.33° (sample SD 2.08°). After deterioration, five 1.50 s left turns averaged 86.2°. Under the subsequent altered painter-tape condition, 0.70 s left turns averaged 89.5°; 0.90 s right turns averaged 86.5°; and 2.20 s right turns averaged 185°. The strong difference between right-angle rates at 90° and 180° motivates interpolated empirical durations. **These groups are not interchangeable** because the drivetrain condition changed. Rotation-center displacement was subsequently estimated from four action-level turn observations; nevertheless, the compensated full route remained inaccurate.
- **Repeatability:** The completed rotation trials show non-negligible variation even for identical normalized inputs; notably the 1.00 s right-turn sample SD was 11.0° across three runs. The 0.95 s left-turn sample SD was 4.0°. Discuss experimental measurement uncertainty and lack of feedback as possible contributors; do not claim any specific cause was isolated.
- **Sources of uncertainty:** Wheel slip/skid steering, motor dead zone, startup response, floor friction/compliance, battery voltage under load (not measured), individual drivetrain condition, painter-tape alteration, command-timing variability, and manual measurement error. The intermittent right-rear-wheel problem remains unresolved; taped-condition yaw measurements do not verify safe or reproducible drivetrain operation.
- **Long-route accumulation:** The two-segment run showed roughly 5–15 cm uncommanded lateral displacement during a left turn. The full route was executed, with visibly increasing positional error and an approximate 0.24 m y shortfall before waypoint 4's final turn; the exact final endpoint error is still unmeasured. Possible contributors are non-centered turning, unequal wheel response, yaw error, and translation-duration mismatch. Subsequent action-level measurements provided signed x/y turn shifts, allowing an effective-pivot model to be fitted and enabled provisionally in the revised controller; it still needs physical validation.
- **Design trade-off:** Turn–drive–turn is easy to calibrate and matches pose waypoints, but it assumes nearly centered turns and accumulates error without feedback. Forward-curved maneuvers were briefly tried as a workaround on carpet but were not repeatable enough to justify replacing the existing planner. An optional turn-displacement model was added for targeted follow-up instead.
- **Potential improvements:** Camera/landmark feedback, wheel encoder data if available, refined command characterization, and better velocity/acceleration profiling. These were not required for the present open-loop demonstration.

## 7. Conclusion

The assigned waypoint sequence was parsed and commanded on the real Wave Rover/Rubik Pi using calibrated turn–drive–turn motions. Independent calibration gave good short forward performance in the early mechanical configuration, and the first two physical waypoint transitions worked. However, the action-level diagnostic revealed **turn-center translations (roughly 6–13 cm for observed individual turns), variable yaw response, and forward travel below the 0.8 m target**. The resulting waypoint-4 checkpoint error was approximately **0.320 m** in the measured diagnostic. The subsequent open-loop effective-pivot compensation did **not** prevent accumulating orientation error; at `(2,2)` the robot departed in an incorrect direction toward `(1,2)` instead of `(1,1)`. This is an unsuccessful *accurate-tracking* result, despite successful execution of the code and a recognizable initial full-route traversal.

The main lesson is that modeling turn-induced translation alone cannot eliminate accumulating **heading** uncertainty. Consistent drivetrain operation and real pose/yaw feedback would be required to make the full trajectory reliably accurate on this surface. The supplied YouTube link documents an example experiment; numerical final-route endpoint accuracy was not measured.

## 8. How to run the code (reproducibility)

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

**Terminal 2 — waypoint controller (dry-run; Mac or Ubuntu, no ROS required):**

```bash
cd ~/ros2_ws/src/rubikpi_ros2/robot_control
python3 robot_control/waypoint_controller.py --dry-run
python3 robot_control/waypoint_controller.py --dry-run --segments 2
```

The dry-run reads `config/waypoints.txt` and `config/calibration.json` relative to the Python source's package directory. Equivalent explicit file paths are accepted with `--waypoints PATH --calibration PATH`. Physical execution is **NOT currently recommended with a right-rear wheel behaving abnormally**. Only after the drivetrain is checked and calibration validated, the basic one-leg format is:

```bash
python3 robot_control/waypoint_controller.py --execute --segments 1 --hardware-checked
```

A full run requires reliable left and right turns. The latest `calibration.json` marks **both left and right timing tested** in the painter-tape configuration. `right.verified_current_hardware` remains `false` because the suspected wheel fault and tape arrangement have not been verified mechanically safe; **timing calibration and hardware safety are distinct**. Accordingly, `--allow-unverified-right` is *not* required for a calibrated full run using the latest config. Hardware safety is not certified by the code. The `--hardware-checked` option is an operator attestation after physical inspection, not a hardware diagnostic. The revised controller also accepts `--pause-at-waypoints` to stop at each waypoint leg during a supervised run. Do not execute if tape restricts a rotating part or a wheel binds, stalls, overheats, or grinds. The user must supervise physical movement and keep power-off access available. Stop if a wheel stalls, grinds, or jerks.

**Optional ROS console entry point:** In `setup.py` under `entry_points={'console_scripts': [...]}`, add `'waypoint_controller = robot_control.waypoint_controller:main',` and rebuild with `colcon build --symlink-install --packages-select robot_control`. This is optional if running the script with `python3`.

Ensure both terminals use the same ROS domain; run only one publisher that sends motion commands at a time. The robot must start in a known physical pose. Do not run unattended or near edges/obstacles. **Physical route-testing record:** `--segments 1`, `--segments 2`, and **one full six-segment uncompensated run** have been completed. The full run had significant unmeasured cumulative position drift. Further operation remains conditional on verifying that the right-rear wheel and painter-tape arrangement do not restrict moving parts or create mechanical strain.

**Supervised checkpoint test (after physical inspection):**

```bash
python3 robot_control/waypoint_controller.py --execute --segments 3 --hardware-checked --pause-at-waypoints
```

This stops after each waypoint leg and waits for Enter (`q` aborts). For diagnosing the waypoint-4 error, use the more detailed **action-level pause** described below. Pauses allow external measurement but do not supply real-time feedback to the controller. **Action-level diagnostic through waypoint 4 (after drivetrain safety inspection):**

```bash
cd ~/ros2_ws/src/rubikpi_ros2/robot_control
python3 robot_control/waypoint_controller.py --dry-run --segments 4
python3 robot_control/waypoint_controller.py --execute --segments 4 --hardware-checked --pause-after-actions
```

At each pause, measure the center and body heading using the same floor axes. Focus particularly on **before and after leg 4's initial left turn**, and **before and after its 0.8 m forward command**. A `--pause-after-actions` command does not reposition the robot or alter the open-loop model. Abort if a wheel binds, stalls, or tape snags. Turn-center compensation has now been fitted from four measured full-run turns and is enabled provisionally. Verify its next trajectory with a supervised short test; fitted shifts are not the same as direct perfect 90° measurements.

A full continuous run should only be attempted after the drivetrain is mechanically safe and shorter tests are reliable. The compensated physical attempt was not accurate, so the revised model must not be presented as validated.

## 9. Submission contents

Include `report.pdf` (exported from this document), this Markdown source, the optional `README_HW1_SKIM.md`, and the runnable `robot_control` ROS package source including the course-provided `setup.py` and `package.xml`, `waypoint_controller.py`, `calibrate_motion.py`, `config/waypoints.txt`, `config/calibration.json`, and `calibration/trials.csv` as appropriate. Exclude generated `build/`, `install/`, `log/`, and Python caches. Link the **unlisted** YouTube demonstration in the PDF; the video itself does not need to be in the ZIP.

### Latest corrected plan: reproduction

With the revised config, the planner's fitted turn-center translations are enabled. It prints intermediate estimated chassis-center coordinates so that potentially unsafe excursions beyond the 0–1.6 m map rectangle can be identified before driving. Run `python3 robot_control/waypoint_controller.py --dry-run --segments 2` before a physically supervised first-two-leg verification. The original calibration log `calibration/trials.csv` is preserved. The video link is provided above; the final full-route pose error remains unmeasured.

### Boundary-aware dry run and tiny-turn commands

The revised planner suppresses turns smaller than 3 degrees (duration too small to be repeatable with a nominal 20 Hz hardware-command loop). It prints intermediate predicted robot CENTER coordinates after each motion. The `--workspace XMIN YMIN XMAX YMAX` option additionally checks predicted intermediate center positions against an explicitly specified physically measured safe rectangle **relative to the starting robot center**. It refuses physical execution if any predicted action endpoint is outside that region, but cannot guarantee clearance between action endpoints or account for the robot's size and model error.

**Important bounded-workspace finding:** At scale 0.8, the shift-compensated trajectory has a predicted center near `(0.77, -0.093) m` immediately after the first left turn. Other intermediate centers can also slightly exceed the nominal 0–1.6 m map bounds. Merely compensating mathematically does not make this feasible inside a strict 1.6 m center-coordinate square with origin exactly at its corner. If the instructor allows a smaller scale, an example dry-run-only alternative is `--scale 0.7`, with the physical starting origin shifted inside the test area to create clearance on every side. Any shift or scale must be documented; changing it does not modify the original waypoints file. The actual rover footprint and uncertain motion require extra space beyond center-point bounds.

Example dry runs:

```bash
python3 robot_control/waypoint_controller.py --dry-run --segments 2
python3 robot_control/waypoint_controller.py --dry-run --scale 0.7
# Example center bounds if physical origin lies 0.1m inside each edge:
python3 robot_control/waypoint_controller.py --dry-run --scale 0.7 --workspace -0.1 -0.1 1.5 1.5
```

**Do not assume the pre-fault or taped-wheel drivetrain is safe to operate:** Do not execute with a wheel restricted by tape, binding, grinding, or repeatedly stalling. Open-loop mathematical predictions and a successful ROS dry-run cannot certify hardware safety.
