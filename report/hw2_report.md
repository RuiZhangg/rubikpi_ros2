# CSE 276A Homework 2 — Vision-Based Closed-Loop Localization

**Name:** Rui Zhang  
**Video:** [YouTube unlisted link]  
**Code:** ROS 2 packages submitted with this report

## 1. System and Experimental Setup

The goal of this assignment is to extend the open-loop waypoint controller from HW1 into a closed-loop controller using visual landmark feedback. The robot follows the required route

\[
(0,0,0)\rightarrow(1,0,0)\rightarrow(1,2,\pi)\rightarrow(0,0,0),
\]

while using AprilTag observations to correct drift in its internal pose estimate.

The system contains four main stages: camera capture, AprilTag detection, pose estimation, and waypoint control. The waypoint controller maintains an estimate of the robot pose \((x,y,\theta)\). Between visual observations, this estimate is propagated using commanded motion. When valid AprilTags are detected, their known world poses are used to estimate the robot pose and correct accumulated dead-reckoning error.

![Closed-loop controller](figures/controller_diagram.png)

| Component | Input | Output / role |
|---|---|---|
| `robot_vision_camera` | Camera | Publishes 1280×720 images and camera calibration |
| `image_proc` | Raw image + camera calibration | Produces an undistorted/rectified image for landmark detection |
| `apriltag_ros` | Rectified image + camera intrinsics | Detects `tag36h11` landmarks and estimates tag pose using PnP |
| `[hw2 controller node]` | Tag poses + tag map + motion estimate | Estimates robot pose, applies visual corrections, and generates waypoint commands |
| `velocity_mapping` | `/cmd_vel` | Converts linear/angular velocity commands to left/right motor commands |
| `motor_control` | Left/right motor commands | Sends commands to the rover |

### Camera Calibration

The Rubik Pi camera was calibrated before landmark localization because its raw image showed noticeable lens distortion. I printed a checkerboard with **9 × 6 inner corners** and **23 mm squares** on A4 paper and mounted it flat on a rigid plastic board. I collected **30** calibration images at 1280×720 while moving the board across the image, changing distance, and tilting it to provide coverage of the center, edges, and corners.

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

These parameters were saved in the camera configuration used by the ROS camera node. The calibrated image size is **1280 × 720**. I verified the integration through `/camera/camera_info`; the running node published the same intrinsic matrix and distortion coefficients, confirming that the new calibration file was loaded correctly.

For landmark detection, the raw image is rectified using `image_proc` before being passed to `apriltag_ros`. This keeps the image geometry consistent with the rectified camera projection model used for AprilTag pose estimation.

### Camera-to-Robot Transform and Motion Calibration

The fixed camera-to-robot transform was measured from the physical mounting geometry. The camera origin was approximately **[___ m forward/back]**, **[___ m left/right]**, and **[___ m above]** the robot center, with orientation **[___]** relative to the vehicle frame.

The rover's motion mapping was also recalibrated before the final experiment. The final parameters were **[briefly list only parameters actually changed]**. This calibration is used for short-term dead reckoning, while visual observations correct accumulated drift.

### Coordinate Frames and Start Pose

The **world frame** has its origin at the first waypoint. \(+x\) points **[direction in room]**, \(+y\) points **[direction in room]**, and positive yaw is counterclockwise. The robot starts at

\[
(x,y,\theta)=(0,0,0).
\]

The **vehicle frame** is centered at **[robot center definition]**, with \(+x\) pointing forward and \(+y\) pointing left. The **camera optical frame** has \(+x\) to the image right, \(+y\) downward, and \(+z\) forward from the camera. The fixed camera-to-vehicle transform is used to convert AprilTag measurements into robot poses in the world frame.

---

## 2. Landmarks and Pose Estimation

### (b) Landmark Selection and Placement

I used **AprilTags from the `tag36h11` family**. The ROS detector was configured to use PnP pose estimation. Each physical tag was assigned a unique ID, and its measured detection-edge size was entered in the detector configuration before pose estimation.

| Tag ID | Detection-edge size | World pose \((x,y,\theta)\) | Placement | Reason |
|---:|---:|---|---|---|
| [ ] | [ ] m | [ ] | [ ] | [ ] |
| [ ] | [ ] m | [ ] | [ ] | [ ] |
| [ ] | [ ] m | [ ] | [ ] | [ ] |
| [ ] | [ ] m | [ ] | [ ] | [ ] |

The tags were mounted on rigid surfaces around the test area so that useful observations were available before and after major turns. Their placement balances visibility and pose accuracy: very close tags can leave the camera field of view or be observed at extreme angles, while distant tags occupy fewer pixels and produce noisier pose estimates. Tags were therefore placed especially near **[turns / long segments / waypoint regions actually used]**, where open-loop drift was most likely to affect the route.

### (c) Robot Pose Estimation and Measurement Noise

For each detected tag, `apriltag_ros` estimates the transform between the camera and tag using the calibrated camera model and the known physical tag size. Using the known tag pose in the world and the fixed camera-to-robot transform, I compute a robot pose estimate in the world frame.

To reduce measurement noise, I **[describe actual method: reject large residuals / average recent estimates / combine multiple visible tags / use closest valid tag]**. I chose this method because **[brief motivation tied to observed measurements]**. Visual corrections are applied **[continuously / at selected points / while stopped]**, reducing accumulated dead-reckoning error without allowing isolated noisy measurements to cause large control changes.

### (d) No-Landmark Behavior

When no landmark is visible, the robot **[describe actual behavior]**. During these intervals it can continue for a short period using its internal motion estimate. **[If applicable: near a waypoint or after ___ s without a valid observation, the robot performs ___ to reacquire a tag.]**

This behavior is necessary because the camera cannot observe every landmark continuously along the complete route.

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

| Waypoint | Target pose \((x,y,\theta)\) | Estimated pose at arrival | Translation error | Rotation error |
|---|---|---|---:|---:|
| 1 | \((0,0,0)\) | [ ] | [ ] m | [ ] rad / [ ]° |
| 2 | \((1,0,0)\) | [ ] | [ ] m | [ ] rad / [ ]° |
| 3 | \((1,2,\pi)\) | [ ] | [ ] m | [ ] rad / [ ]° |
| 4 | \((0,0,0)\) | [ ] | [ ] m | [ ] rad / [ ]° |

### (i) Estimated 2D Trajectory

The controller logs its internal pose estimate throughout the run at approximately **[0.5 / 1.0] s** intervals. During periods without a visible AprilTag, the plotted pose comes from the controller's dead-reckoning estimate; after valid landmark observations, the corrected pose is used. Figure 2 plots these estimates in the world frame together with the required waypoints.

### (j) Overall Performance

The final system **[briefly summarize success]**. Compared with open-loop motion, visual feedback was most useful for **[correcting heading drift after turns / reducing accumulated error on long segments / etc.]**. The largest remaining source of error was **[camera pose noise / wheel slip / imperfect camera extrinsic / intermittent tag visibility / controller overshoot]**.

The main limitation of the implementation is **[___]**. With more time, I would improve **[one or two concrete items only]**. Despite these limitations, the experiment demonstrates the intended closed-loop behavior: landmark observations are used to estimate the robot pose and correct motion errors while following the required waypoint sequence.

---

## Final Run Summary

- Final video: **[link]**
- Camera calibration: **30 images, 1280×720, RMS 0.2699 px**
- AprilTag family: **tag36h11**
- Pose estimation method: **PnP**
- Number of AprilTags used: **[___]**
- Final route completion: **[successful / description]**
- Total translational distance: **[___ m]**
- Final endpoint translation error: **[___ m]**
- Final endpoint rotation error: **[___ rad / ___°]**
- Trajectory log: **[filename]**