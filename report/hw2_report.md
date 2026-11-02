# CSE 276A Homework 2 — Vision-Based Closed-Loop Localization

**Name:** Rui Zhang  
**Video:** [YouTube unlisted link]  
**Code:** ROS 2 packages submitted with this report

## 1. System and Experimental Setup

The goal of this assignment is to extend the open-loop waypoint controller from HW1 into a closed-loop controller using visual landmark feedback. The required waypoint sequence is

\[
(0,0,0)\rightarrow(1,0,0)\rightarrow(1,2,\pi)\rightarrow(0,0,0).
\]

As in HW1, my available physical test area uses a scale of **0.8 m per course coordinate unit**. Therefore, the physical waypoint positions used by the controller are

\[
(0,0,0)\rightarrow(0.8,0,0)\rightarrow(0.8,1.6,\pi)\rightarrow(0,0,0).
\]

The system contains camera capture, image rectification, AprilTag detection, pose estimation, and waypoint control. The waypoint controller maintains an estimate of the robot pose \((x,y,\theta)\) in meters and radians. Between visual observations, this estimate is propagated using commanded motion. When a valid AprilTag is detected, its known world pose is used to estimate the robot pose and correct accumulated dead-reckoning error.

![Closed-loop controller](figures/controller_diagram.png)

| Component | Input | Output / role |
|---|---|---|
| `robot_vision_camera` | Camera | Publishes 1280×720 images and camera calibration |
| `image_proc` | Raw image + camera calibration | Produces an undistorted/rectified image |
| `apriltag_ros` | Rectified image + camera intrinsics | Detects `tag36h11` landmarks and estimates tag pose using PnP |
| `[hw2 controller node]` | Tag poses + landmark map + motion estimate | Estimates robot pose, applies visual corrections, and generates waypoint commands |
| `velocity_mapping` | `/cmd_vel` | Converts linear/angular velocity commands to left/right motor commands |
| `motor_control` | Left/right motor commands | Sends commands to the rover |

### Camera Calibration

The Rubik Pi camera was calibrated before landmark localization because its raw image showed noticeable lens distortion. I printed a checkerboard with **9 × 6 inner corners** and **23 mm squares** on A4 paper and mounted it flat on a rigid plastic board. I collected **30** calibration images at 1280×720 while moving the board across the image, changing distance, and tilting it to cover the center, edges, and corners.

OpenCV calibration produced an RMS reprojection error of **0.2699 px**. The intrinsic matrix was

\[
K=
\begin{bmatrix}
631.0711 & 0 & 651.9864\\
0 & 632.1226 & 337.2623\\
0 & 0 & 1
\end{bmatrix},
\]

with plumb-bob distortion coefficients

\[
(k_1,k_2,p_1,p_2,k_3)
=
(-0.340540,\ 0.136549,\ 0.000374,\ -0.000483,\ -0.027330).
\]

These parameters were saved in the camera configuration used by the ROS camera node. I verified the integration through `/camera/camera_info`; the running node published the same intrinsic matrix and distortion coefficients. For landmark detection, the raw image is rectified with `image_proc` before being passed to `apriltag_ros`.

### Camera-to-Robot Transform and Motion Calibration

I define `base_link` at the midpoint of the four wheels projected onto the floor, with \(+x\) forward, \(+y\) left, and \(+z\) upward. The fixed camera-to-robot transform was estimated from a stationary AprilTag observation at a measured tag pose relative to the robot. During calibration, the tag center was at approximately \((0.585,0.000,0.144)\) m in the robot frame. Averaging 50 AprilTag measurements produced

\[
{}^{B}t_C=(0.0782,-0.0076,0.1153)\ {\rm m},
\]

with camera orientation

\[
(\phi,\theta,\psi)
=
(-88.42^\circ,-1.11^\circ,-90.47^\circ)
\]

in roll, pitch, and yaw. The translation standard deviations across the 50 samples were approximately \(0.05\) mm, \(0.40\) mm, and \(0.51\) mm in \(x,y,z\). The estimated translation was also close to the hand-measured camera placement of about 8.5 cm forward, centered laterally, and 12.7 cm above the floor, so I used this transform for localization.

The rover's motion mapping was recalibrated before the final experiment. The final parameters were **[fill after motion calibration]**. This calibration is used for short-term dead reckoning, while visual observations correct accumulated drift.

### Coordinate Frames and Start Pose

The **world frame** has its origin at the first waypoint. \(+x\) points in the robot's initial forward direction, \(+y\) points to the robot's initial left, and \(+z\) points upward. Positive yaw is counterclockwise. The robot starts at

\[
(x,y,\theta)=(0,0,0).
\]

The **vehicle frame** is `base_link`, defined at the midpoint of the wheels projected onto the floor. The **camera optical frame** has \(+x\) to image right, \(+y\) downward, and \(+z\) forward. The camera is not level relative to the chassis, so its fixed rotation is explicitly represented by the calibrated `base_link -> camera_frame` transform.

---

## 2. Landmarks and Pose Estimation

### (b) Landmark Selection and Placement

I used **AprilTags from the `tag36h11` family** with PnP pose estimation. Each printed tag has a measured detection-edge size of **0.154 m**, which was entered into the AprilTag configuration.

Before constructing the landmark map, I validated a single tag while the robot was stationary. Moving the tag to camera right made the reported camera-frame \(x\) positive, moving it downward made \(y\) positive, and increasing the distance increased \(z\). Representative detections had translations \((0.290,-0.010,0.562)\), \((-0.038,-0.011,1.033)\), and \((-0.049,-0.014,0.779)\) m, corresponding to camera-to-tag ranges of approximately 0.632 m, 1.034 m, and 0.781 m.

I placed three landmarks around the route so that each major segment should have at least one useful landmark in view. The current map is:

| Tag ID | Approx. course-coordinate pose | Approx. physical \(x,y\) | Face yaw | Center height |
|---:|---|---|---:|---:|
| 1 | \((2.5,-0.025)\) | \((2.000,-0.020)\) m | \(\pi\) | 0.234 m |
| 0 | \((1,2.5)\) | \((0.800,2.000)\) m | \(-\pi/2\) | 0.152 m |
| 2 | \((0,-0.5)\) | \((0.000,-0.400)\) m | \(\pi/2\) | 0.154 m |

The landmark coordinates were measured in the physical world frame before the final experiment. The placement was chosen so that tag 1 covers the first segment and first turn, tag 0 covers the long positive-\(y\) segment and upper waypoint, and tag 2 provides visibility near the start/return region.

### (c) Robot Pose Estimation and Measurement Noise

For each detected tag, `apriltag_ros` estimates the transform from the camera to that tag using the calibrated camera model and known 0.154 m tag size. The fixed `base_link -> camera_frame` transform converts this observation to the robot frame. Combining it with the corresponding known world-to-tag transform gives an independent estimate of the robot pose in the world frame.

To reduce measurement noise, the localization node rejects AprilTag transforms older than **1.0 s**. If more than one mapped tag is visible, it uses the **closest valid tag**, since a nearer tag occupies more image pixels and generally gives a more stable PnP estimate. The localization node publishes the resulting planar world pose on `/hw2/vision_pose`.

I validated the complete world-localization chain by manually placing the robot at four known poses. At \((0,0,0)\), the estimate was \((-0.038,0.060,-2.9^\circ)\); at \((0.8,0,0)\), it was \((0.806,0.007,-0.5^\circ)\); at \((0.8,1.6,\pi/2)\), it was \((0.803,1.634,92.7^\circ)\); and at \((0.8,1.6,-\pi/2)\), it was \((0.803,1.636,-90.1^\circ)\). The corresponding translation errors were approximately **7.1 cm, 0.9 cm, 3.4 cm, and 3.6 cm**. The largest error occurred at the origin while using tag 1 from approximately 2.06 m away, consistent with lower pose accuracy at longer range.

For closed-loop control, fresh visual poses are blended with the dead-reckoned state rather than applied as hard resets. The controller uses a weight of 0.50 for visual position and 0.60 for visual yaw, and rejects single visual updates that differ from the current state by more than 0.45 m or 45°. This reduces abrupt state jumps while still correcting accumulated drift.

### (d) No-Landmark Behavior

When no mapped landmark is visible, the localization node publishes no new visual correction. The controller continues using dead reckoning from its commanded linear and angular velocities until another valid landmark observation becomes available. This allows short gaps in landmark visibility without stopping the robot.

---

## 3. Controller Performance

### (f) Path Smoothness and Correction Behavior

Figure 2 shows the estimated trajectory of the final run. Along the first segment, the robot **[under-corrected / over-corrected / stayed close to the reference]**, with corrections of approximately **[___]**. Near the second waypoint, **[describe behavior]**. On the return segment, **[describe behavior]**.

Overall, the path was **[smooth / moderately oscillatory / etc.]**. The trajectory shows **[whether corrections converged toward the reference or alternated across it]**. The controller used **[small/frequent or larger/infrequent]** corrections because **[brief reason]**. By each waypoint, **[describe remaining error and any final correction]**.

![Estimated trajectory](figures/trajectory.png)

### (g) Total Distance Travelled

I estimated translational distance by summing displacement of the robot center between successive logged pose estimates:

\[
D=\sum_k \sqrt{(x_{k+1}-x_k)^2+(y_{k+1}-y_k)^2}.
\]

The final estimated translational distance was **[___ m]**. In-place rotation is excluded from this value.

### (h) Waypoint Errors

Waypoint error was computed from the robot's onboard pose estimate when the controller declared each waypoint reached. Translation error is the Euclidean position difference from the target, and rotation error is the wrapped yaw difference. Floor waypoint markers and the final video provide a visual check that the robot physically reached the intended locations.

| Waypoint | Physical target pose \((x,y,\theta)\) | Estimated pose at arrival | Translation error | Rotation error |
|---|---|---|---:|---:|
| 1 | \((0,0,0)\) | [ ] | [ ] m | [ ] rad / [ ]° |
| 2 | \((0.8,0,0)\) | [ ] | [ ] m | [ ] rad / [ ]° |
| 3 | \((0.8,1.6,\pi)\) | [ ] | [ ] m | [ ] rad / [ ]° |
| 4 | \((0,0,0)\) | [ ] | [ ] m | [ ] rad / [ ]° |

### (i) Estimated 2D Trajectory

The controller logs its internal pose estimate throughout the run at approximately **[0.5 / 1.0] s** intervals. During periods without a visible AprilTag, the plotted pose comes from dead reckoning; after valid landmark observations, the corrected pose is used. Figure 2 plots these estimates together with the required waypoint sequence.

### (j) Overall Performance

The final system **[briefly summarize success]**. Compared with open-loop motion, visual feedback was most useful for **[correcting heading drift after turns / reducing accumulated error on long segments / etc.]**. The largest remaining source of error was **[camera pose noise / wheel slip / imperfect landmark map / intermittent tag visibility / controller overshoot]**.

The main limitation of the implementation is **[___]**. With more time, I would improve **[one or two concrete items]**. Despite these limitations, the experiment demonstrates the intended closed-loop behavior: landmark observations are used to estimate robot pose and correct motion errors while following the required waypoint sequence.

---

## Final Run Summary

- Final video: **[link]**
- Physical scale: **0.8 m per course coordinate unit**
- Camera calibration: **30 images, 1280×720, RMS 0.2699 px**
- Camera extrinsic translation: **(0.0782, -0.0076, 0.1153) m**
- Camera extrinsic RPY: **(-88.42°, -1.11°, -90.47°)**
- AprilTag family: **tag36h11**
- AprilTag detection-edge size: **0.154 m**
- Number of AprilTags used: **3**
- Final route completion: **[successful / description]**
- Total translational distance: **[___ m]**
- Final endpoint translation error: **[___ m]**
- Final endpoint rotation error: **[___ rad / ___°]**
- Trajectory log: **[filename]**