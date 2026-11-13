# CSE 276A Homework 3 - EKF Simultaneous Localization and Mapping

**Name:** Rui Zhang  
**Video:** TODO[SUBMISSION]: Add the unlisted YouTube link showing one complete
square lap and one complete octagon lap.  
**Code:** ROS 2 packages and analysis tools submitted with this report.

> Status convention: `TODO[ARENA]` requires a physical measurement,
> `TODO[CALIBRATION]` requires a calibration run, `TODO[RESULT]` requires an
> arena experiment, and `TODO[SUBMISSION]` is a final packaging item. These
> labels prevent unmeasured values from being presented as results.

## 1. Objective and system overview

The objective is to implement an extended Kalman filter (EKF) SLAM system that
simultaneously estimates the planar rover pose and the positions of an initially
unknown number of landmarks. The rover begins without a global pose or landmark
map. The online system constructs a map in a local SLAM frame, uses its current
pose estimate for closed-loop motion correction, and later transforms the result
into the independently measured world frame for evaluation.

The rover has no wheel encoders and its IMU is not reliable enough to use as a
primary motion measurement. Prediction therefore uses calibrated commanded
motion increments with relatively large process uncertainty. AprilTag
observations carry most of the estimation weight. Because rectified AprilTag
detections arrive below 1 Hz, the baseline controller moves about 5 cm, stops,
waits for a fresh observation, and then chooses the next action.

TODO[RESULT]: Add a final block diagram showing camera, rectification,
AprilTag detection, base-frame observation, EKF-SLAM, closed-loop controller,
velocity mapping, and motor control.

## 2. Environment, landmarks, and measured ground truth

### 2.1 Arena

The available room is approximately 10 ft 10 in square, or approximately
3.302 m per side. This exceeds the assignment's 8 ft minimum and provides enough
travel distance for motion uncertainty and loop-closure improvement to be
visible.

TODO[ARENA]: Replace the approximate dimension with four individually measured
wall lengths. Record the usable interior boundary, obstacles, carpet condition,
and coordinate-frame definition.

### 2.2 Landmark choice and placement

Eight unique `tag36h11` AprilTags are used, with two tags on each wall. Every
tag has a measured detection-edge size of 0.154 m. Unique IDs make data
association deterministic while the EKF still discovers landmarks dynamically;
the state is not preallocated with eight landmarks.

The tags are mounted at approximately one-third/two-thirds wall spacing, upright
at the wall plane and facing inward. IDs increase counterclockwise from the
right/east tag on the north wall. Tags 4 and 5 have center height 0.124 m; all
others have center height 0.210 m. The unequal height is retained as a measured
property and will be checked during the visibility survey.

| Tag ID | Wall | World x (m) | World y (m) | Center height (m) | Face direction | Notes |
|---:|---|---:|---:|---:|---|---|
| 0 | North, east/right | TODO[ARENA] | TODO[ARENA] | 0.210 | inward (`-y`) | Mounted |
| 1 | North, west/left | TODO[ARENA] | TODO[ARENA] | 0.210 | inward (`-y`) | Mounted |
| 2 | West, north/upper | TODO[ARENA] | TODO[ARENA] | 0.210 | inward (`+x`) | Mounted |
| 3 | West, south/lower | TODO[ARENA] | TODO[ARENA] | 0.210 | inward (`+x`) | Mounted |
| 4 | South, west/left | TODO[ARENA] | TODO[ARENA] | 0.124 | inward (`+y`) | Mounted |
| 5 | South, east/right | TODO[ARENA] | TODO[ARENA] | 0.124 | inward (`+y`) | Mounted |
| 6 | East, south/lower | TODO[ARENA] | TODO[ARENA] | 0.210 | inward (`-x`) | Mounted |
| 7 | East, north/upper | TODO[ARENA] | TODO[ARENA] | 0.210 | inward (`-x`) | Mounted |

TODO[ARENA]: Add the top-down ground-truth map with wall dimensions, tag IDs,
tag centers, path start, world axes, square path, and octagon path.

### 2.3 Coordinate frames and unknown start pose

The world frame is defined from physical measurements of the arena. The online
filter is not given that frame. It fixes its gauge freedom by defining the
initial `base_link` pose as the origin of a local SLAM frame:

\[
\hat{\mathbf{x}}_{r,0}=[0,0,0]^T.
\]

This convention does not reveal the robot's global start pose. For evaluation
only, the measured initial world pose
\((x_{w0},y_{w0},\theta_{w0})\) is used to transform estimated robot and
landmark positions into the world frame.

A physical start-position and yaw reference is used only to repeat trials and
measure that offline transform. The rover does not observe floor marks, and no
measured path-vertex coordinates or landmark ground truth are supplied to the
online estimator or controller. Square and octagon waypoint errors are corrected
from the current EKF pose estimate.

TODO[ARENA]: Record the physical start pose and explain how it was measured.

## 3. EKF-SLAM algorithm

### 3.1 State vector \(\mathbf{x}\)

The state contains the planar robot pose followed by the 2-D position of every
landmark discovered so far:

\[
\mathbf{x}=
[x_r,y_r,\theta_r,l_{1x},l_{1y},\ldots,l_{Nx},l_{Ny}]^T.
\]

The dictionary from AprilTag ID to state indices is initially empty. Therefore
the filter does not assume that eight landmarks exist. Landmark orientation is
not included in the baseline state because tag-center position is sufficient for
planar EKF-SLAM and is less sensitive than PnP orientation at long range.

### 3.2 Control and motion model

One completed motion primitive supplies

\[
\mathbf{u}_k=[\Delta s_k,\Delta\theta_k]^T,
\]

where \(\Delta s\) is a commanded straight displacement and
\(\Delta\theta\) is a commanded in-place rotation. The implemented midpoint
model is

\[
\begin{aligned}
\phi_k &= \theta_k+\frac{\Delta\theta_k}{2},\\
x_{k+1} &= x_k+\Delta s_k\cos\phi_k,\\
y_{k+1} &= y_k+\Delta s_k\sin\phi_k,\\
\theta_{k+1} &= \operatorname{wrap}(\theta_k+\Delta\theta_k).
\end{aligned}
\]

Landmark states remain unchanged during prediction.

### 3.3 System matrix \(F\) and control matrix \(G\)

For the robot block,

\[
F_r=
\begin{bmatrix}
1&0&-\Delta s\sin\phi\\
0&1& \Delta s\cos\phi\\
0&0&1
\end{bmatrix},
\qquad
G_r=
\begin{bmatrix}
\cos\phi&-\frac{\Delta s}{2}\sin\phi\\
\sin\phi& \frac{\Delta s}{2}\cos\phi\\
0&1
\end{bmatrix}.
\]

The full \(F\) is identity except for its upper-left robot block. The full
\(G\) has \(G_r\) in its first three rows and zeros in landmark rows. Covariance
prediction is

\[
P^- = FPF^T+GMG^T,
\]

where \(M\) is the calibrated covariance of commanded translation and rotation.

### 3.4 Measurement vector \(\mathbf{z}\) and model

For observed tag \(i\), the camera-to-tag PnP result and calibrated
`base_link -> camera_frame` transform produce the tag center in the robot plane:

\[
\mathbf{z}_i=[z_{ix},z_{iy}]^T.
\]

Let \(\delta_x=l_{ix}-x_r\), \(\delta_y=l_{iy}-y_r\),
\(c=\cos\theta_r\), and \(s=\sin\theta_r\). The prediction is

\[
h_i(\mathbf{x})=
\begin{bmatrix}
c\delta_x+s\delta_y\\
-s\delta_x+c\delta_y
\end{bmatrix}.
\]

### 3.5 Measurement matrix \(H\)

Only the robot pose and the observed landmark have nonzero columns:

\[
H_{i,r}=
\begin{bmatrix}
-c&-s&h_{iy}\\
s&-c&-h_{ix}
\end{bmatrix},
\qquad
H_{i,l_i}=
\begin{bmatrix}
c&s\\
-s&c
\end{bmatrix}.
\]

The innovation is \(\mathbf{y}=\mathbf{z}-h_i(\hat{\mathbf{x}}^-)\), with
\(S=HP^-H^T+R\) and \(K=P^-H^TS^{-1}\). The implementation uses the Joseph
covariance update for numerical stability and symmetrizes the result.

### 3.6 Initial state and covariance

The initial local robot pose is fixed at the SLAM-frame origin. A very small
robot covariance fixes this otherwise unobservable global translation and
rotation gauge. The landmark map is empty, so the initial state has three
elements and the initial covariance is 3 by 3.

TODO[CALIBRATION]: State the final numerical initial covariance and justify it.

### 3.7 Process noise \(Q\) / control noise \(M\)

The rover has no encoders, its IMU is not trusted, the floor is low carpet, and
turning is inconsistent. Motion uncertainty is therefore measured from repeated
5 cm translations and repeated turn pulses on the actual arena carpet. The
sample covariance of realized \([\Delta s,\Delta\theta]\) errors becomes
\(M\), which is mapped into state space by \(G\).

TODO[CALIBRATION]: Report trial count, raw-error plots, sample covariance, and
the final numerical values used.

### 3.8 Measurement noise \(R\)

Measurement noise is measured with a stationary robot observing tags at several
ranges and view angles after image rectification. The baseline implementation
allows standard deviation to grow with range because distant tags occupy fewer
pixels and produced larger HW2 pose errors.

TODO[CALIBRATION]: Report sample counts, tested ranges/angles, covariance, and
the final range-dependent model.

A preliminary stationary checkpoint placed tag 0 nearly front-on at an observed
range of approximately 0.934 m. Detections had hamming distance 0 and decision
margin approximately 175. The first observation initialized the landmark and
the subsequent inspected observations were accepted; their NIS values were far
below the 9.21 gate. Robot-frame measurements remained near 0.907 m forward and
-0.0363 m left. This demonstrates short-range repeatability and validates the
TF direction, but it does not measure absolute bias or performance at long range
and oblique views. Therefore it is not used alone to reduce `R`.

The subsequent stationary range/angle survey produced the following accepted-
sample statistics. Each run was centered independently per visible tag before
pooling, so the table measures random repeatability rather than absolute bias.

| Nominal pose | Visible IDs | Accepted samples | Pooled forward std. (m) | Pooled left std. (m) |
|---|---|---:|---:|---:|
| 0.75 m, front-on | 0 | 46 | 0.000093 | 0.000024 |
| 1.50 m, front-on | 0, 1 | 1080 | 0.004801 | 0.003628 |
| 2.25 m, front-on | 0, 1, 7 | 706 | 0.003457 | 0.002017 |
| 1.50 m, +30 degrees | 0, 1 | 292 | 0.001342 | 0.000571 |
| 1.50 m, -30 degrees | 0, 7 | 198 | 0.004222 | 0.003251 |
| 1.50 m, +45 degrees | 1, 2 | 182 | 0.007608 | 0.004698 |
| 1.50 m, -45 degrees | 7 | 108 | 0.000201 | 0.000104 |

The largest individual-tag standard deviation occurred for tag 2 in the
nominal +45-degree run: approximately 0.00955 m forward and 0.00630 m left.
The strong variation by tag/view and correlated covariance in several runs
support retaining a conservative, range-dependent `R`. TODO[CALIBRATION]: Add
rejection rates and tape-measured pose errors from the raw CSVs, then select the
final numerical model. A candidate model for moving trials is
`sigma(r)=0.010+0.005r` m, but it is not yet the final report value.

### 3.9 First observation of a new landmark

For a new ID, the landmark is initialized by transforming its robot-frame
measurement into the SLAM frame:

\[
\begin{bmatrix}l_x\\l_y\end{bmatrix}
=
\begin{bmatrix}x_r\\y_r\end{bmatrix}
+R(\theta_r)\mathbf{z}.
\]

The covariance is augmented using Jacobians with respect to the robot state and
measurement. Cross-covariance with all existing states is preserved; the new
landmark is not added as an independent diagonal block.

### 3.10 Landmark outside the field of view

An unobserved landmark remains in the state and is unchanged by the motion
model. No measurement row is created for it. Its correlation with the uncertain
robot pose is retained in the full covariance.

### 3.11 Previously observed landmark reappearing

The unique tag ID selects its existing state block. The observation is subjected
to detector-quality and normalized-innovation gates. If accepted, a normal EKF
correction updates the robot, that landmark, and correlated landmarks. This is
the mechanism that produces loop closure and should reduce uncertainty on later
laps.

### 3.12 Implementation and pre-arena verification

The mathematical filter is isolated in
`robot_control/robot_control/ekf_slam.py`; it has no ROS dependency. The ROS
wrapper in `hw3_slam_node.py` time-matches AprilTag detections with
`base_link -> tag` transforms, applies detector-quality and innovation gates,
and logs every prediction, accepted update, rejection, state, and marginal
covariance. The controller is implemented separately in `hw3_controller.py`, so
the estimator does not contain path-specific logic.

Before arena testing, automated tests numerically differentiated the motion and
measurement functions and matched the analytic \(F\), \(G\), and \(H\)
Jacobians. Tests also checked dynamic state augmentation, nonzero
cross-covariance, stationary landmark prediction, outlier rejection, covariance
symmetry/positive semidefiniteness, square/octagon closure, and a synthetic
square loop with biased motion. At the pre-arena implementation checkpoint, all
11 offline tests passed with Python warnings treated as errors. TODO[RESULT]:
Replace this count with the final submission-commit test result if it changes.

## 4. Closed-loop trajectory strategy

### 4.1 Stop-observe-update motion

The controller separates translation and rotation. It issues one approximately
5 cm straight motion or one calibrated turn pulse, stops, submits that command
to the EKF prediction, and waits for a new accepted landmark update before the
next decision. This strategy trades runtime for more observations, less motion
blur, and safer behavior with unreliable open-loop motion.

### 4.2 Square trajectory and choice of \(n\)

The initial survey path uses \(n=1.8\) m, centered in the nominal 3.302 m
arena. Its proposed vertices are `(0.751,0.751)`, `(2.551,0.751)`,
`(2.551,2.551)`, and `(0.751,2.551)` m. TODO[ARENA]: Confirm or revise \(n\)
after testing tag range, corner visibility, rover clearance, and usable floor
area. Explain the final selection against accumulated motion uncertainty.

### 4.3 Eight-edge octagon and choice of \(m\)

The initial survey path uses \(m=0.75\) m, producing a centered footprint about
1.811 m square. Its eight nominal vertices are documented in
`docs/arena_setup.md`. TODO[ARENA]: Confirm or revise \(m\) after the visibility
survey and explain the tradeoff between 45-degree turn error, arena coverage,
tag visibility, and comparability with the square trajectory.

### 4.4 No-fresh-observation behavior

The controller stops when a completed primitive is not followed by a fresh
accepted observation before the timeout. After translation it remains stopped;
blind translation is disabled. If a tag leaves view during an intended turn, it
may continue calibrated turn pulses only toward that same target yaw while
searching for the next tag. It stops at the target heading if no observation has
appeared. TODO[RESULT]: Describe the recovery/search behavior actually exercised
in the final arena run.

## 5. Results

All plotted trajectories must come from EKF state logs, not visual estimates.
Landmarks are plotted with 95% position-covariance ellipses.

### 5.1 Square - one lap

TODO[RESULT]: Add trajectory/map plot, landmark covariance ellipses, completion
time, number of observations, final start-closure error, and average landmark
error.

### 5.2 Square - multiple laps

TODO[RESULT]: Add the multi-lap plot and quantify changes in landmark error,
robot closure error, and covariance trace by lap.

### 5.3 Octagon - one lap

TODO[RESULT]: Add trajectory/map plot, landmark covariance ellipses, completion
time, number of observations, final start-closure error, and average landmark
error.

### 5.4 Octagon - multiple laps

TODO[RESULT]: Add the multi-lap plot and quantify changes in landmark error,
robot closure error, and covariance trace by lap.

### 5.5 World-frame transform and landmark error

For evaluation, each SLAM-frame point \(p_s\) is transformed with the measured
start pose:

\[
p_w=R(\theta_{w0})p_s+[x_{w0},y_{w0}]^T.
\]

For each mapped tag with ground-truth position \(g_i\), error is
\(e_i=\lVert \hat l_{i,w}-g_i\rVert_2\), and the required average is

\[
\bar e=\frac{1}{N}\sum_{i=1}^{N}e_i.
\]

TODO[RESULT]: Add a per-tag error table and the average for all four runs.

## 6. Comparison and discussion

### 6.1 Square versus octagon maps

TODO[RESULT]: Compare landmark accuracy, covariance, visibility, turn errors,
path closure, and map shape. Explain whether more frequent 45-degree turns help
view diversity or instead add motion error.

### 6.2 Does the map improve over multiple laps?

TODO[RESULT]: Compare numerical errors and covariance between lap one and later
laps. Identify where loop closure caused uncertainty to shrink and any cases
where overconfident noise parameters prevented improvement.

### 6.3 Main limitations and tradeoffs

Discuss at minimum: no encoder odometry, unreliable IMU, low carpet, inconsistent
turning, sub-1-Hz processed vision, tag distance/angle, landmark placement,
stop-and-go runtime, selected \(n\) and \(m\), and EKF linearization.

TODO[RESULT]: Replace this instruction with evidence from the final runs.

## 7. Video and code submission

TODO[SUBMISSION]: Add the unlisted YouTube link. One complete square lap and one
complete octagon lap are shown; multiple-lap evidence is provided in plots.

TODO[SUBMISSION]: Create the code ZIP, extract it into a clean ROS 2 Jazzy
workspace, build it, and record the exact commit hash used for the report.

## Appendix A. Requirement coverage

| Handout item | Report location | Status |
|---|---|---|
| Ground-truth map and two landmarks per wall | Section 2 | TODO[ARENA] |
| Kalman/EKF SLAM | Section 3 | Implemented design; results pending |
| Unknown start pose, landmark positions, and count | Sections 2.3, 3.1, 3.9 | Addressed |
| Square and 8-point octagon | Sections 4, 5 | TODO[ARENA/RESULT] |
| Compare maps and multiple laps | Section 6 | TODO[RESULT] |
| State and measurement vectors | Sections 3.1, 3.4 | Addressed |
| Derive F, G, and H | Sections 3.3, 3.5 | Addressed |
| Initialize state and covariance | Section 3.6 | Final values pending |
| Determine process and measurement noise | Sections 3.7, 3.8 | TODO[CALIBRATION] |
| New, invisible, and reappearing landmarks | Sections 3.9-3.11 | Addressed |
| Estimated trajectory and landmark covariance | Section 5 | TODO[RESULT] |
| Average landmark error | Section 5.5 | TODO[RESULT] |
| Video link | Section 7 | TODO[SUBMISSION] |
| Code ZIP | Section 7 | TODO[SUBMISSION] |
