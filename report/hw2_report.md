# CSE 276A Homework 2 — Vision-Based Closed-Loop Localization

**Name:** Rui Zhang  
**Video:** [YouTube unlisted link]  
**Code:** ROS 2 packages submitted with this report

<!--
Working report template. Replace all [bracketed items] with final measurements/results.
Keep the final report concise (~3 pages). The comments can be deleted before submission.
-->

## 1. System and Experimental Setup

The goal of this assignment was to extend the open-loop waypoint controller from HW1 into a closed-loop controller using visual landmark feedback. The robot followed the required route

\[
(0,0,0)\rightarrow(1,0,0)\rightarrow(1,2,\pi)\rightarrow(0,0,0),
\]

while using AprilTag observations to correct drift in its internal pose estimate.

The system consisted of four main parts: camera capture, AprilTag detection, pose estimation, and waypoint control. The waypoint controller maintained an estimate of the robot pose \((x,y,\theta)\). Between visual observations, this estimate was propagated using the commanded linear and angular velocities. When one or more valid AprilTags were detected, their known world poses were used to estimate the robot pose and correct the accumulated dead-reckoning error.

![Closed-loop controller](figures/controller_diagram.png)

<!--
(a) The figure should show at least:
camera -> AprilTag detector -> pose/localization -> waypoint controller -> /cmd_vel
                                     ^                   |
                                     |                   v
                                  tag map              robot
and the dead-reckoning/prediction path back into localization.
-->

The main ROS 2 components used in the final system were:

| Component | Input | Output / role |
|---|---|---|
| `robot_vision_camera` | Camera | Publishes image and camera calibration topics |
| `apriltag_ros` | Camera image + intrinsics | Detects tags and estimates tag pose relative to the camera |
| `[hw2 controller node]` | Tag poses + tag map + current motion estimate | Estimates robot pose, performs visual corrections, and generates waypoint commands |
| `velocity_mapping` | `/cmd_vel` | Converts linear/angular velocity commands to left/right motor commands |
| `motor_control` | Left/right motor commands | Sends commands to the rover |

### Camera and Motion Calibration

The camera was calibrated before landmark localization using a printed checkerboard mounted flat on a rigid plastic board. The calibration target had **[columns × rows] inner corners** with a measured square size of **[___ m]**. We collected **[___]** calibration views covering the center, edges, different distances, and different orientations of the image.

Final camera calibration:
- Image resolution: **1280 × 720**
- \(f_x =\) **[___]**
- \(f_y =\) **[___]**
- \(c_x =\) **[___]**
- \(c_y =\) **[___]**
- Distortion coefficients: **[___]**
- Calibration/reprojection error, if available: **[___]**

The camera-to-robot transform was estimated from the physical mounting geometry. The camera origin was approximately **[___ m forward/back]**, **[___ m left/right]**, and **[___ m above]** the robot center, with orientation **[___]** relative to the robot frame.

We also recalibrated the rover's motion mapping before the final experiment. The final motion parameters were **[briefly list only the parameters actually changed]**. This calibration was only used for short-term dead reckoning; visual observations were responsible for correcting accumulated drift.

### Coordinate Frames and Start Pose

The **world frame** was defined with its origin at the first waypoint. \(+x\) pointed **[direction in room]**, \(+y\) pointed **[direction in room]**, and positive yaw was counterclockwise. The robot started at

\[
(x,y,\theta)=(0,0,0).
\]

The **vehicle frame** was centered at **[robot center definition]**, with \(+x\) pointing forward and \(+y\) pointing left. The **camera frame** followed the convention used by the camera/AprilTag ROS pipeline. The fixed camera-to-vehicle transform above was used when converting AprilTag measurements into robot poses in the world frame.

---

## 2. Landmarks and Pose Estimation

### (b) Landmark Selection and Placement

We used **AprilTags** from the **[tag family, e.g. tag36h11]** family. Each tag had a measured detection size of **[___ m]** and a unique ID. The tags were mounted on rigid surfaces around the test area so that the robot had useful visual coverage before and after major turns without placing the tags extremely close to the camera.

| Tag ID | World pose \((x,y,\theta)\) | Placement | Reason |
|---:|---|---|---|
| [ ] | [ ] | [ ] | [ ] |
| [ ] | [ ] | [ ] | [ ] |
| [ ] | [ ] | [ ] | [ ] |
| [ ] | [ ] | [ ] | [ ] |

The placement was chosen to balance visibility and pose accuracy. Tags that were too close could leave the camera field of view or produce difficult viewing angles, while tags that were too far away occupied fewer pixels and gave noisier pose estimates. We especially placed tags near **[turns / long segments / waypoint regions actually used]**, where open-loop drift was most likely to affect the route.

### (c) Robot Pose Estimation and Measurement Noise

For each detected tag, the detector provided the transform from the tag frame to the camera frame. Let \(^{W}T_T\) be the known tag pose in the world, \(^{C}T_T\) the measured tag pose in the camera frame, and \(^{R}T_C\) the fixed camera pose in the robot frame. The corresponding robot pose estimate was computed from the transform chain

\[
^{W}T_R
=
^{W}T_T
(^{C}T_T)^{-1}
(^{R}T_C)^{-1}.
\]

<!-- Verify the exact TF direction used in the final implementation and edit this equation if necessary. -->

To reduce measurement noise, we **[describe actual method: e.g. rejected detections beyond a residual threshold / averaged several recent estimates / combined multiple visible tags with weights / used the closest tag]**. We chose this method because **[brief motivation tied to observed noise]**. Visual corrections were applied **[continuously / at selected points / only while stopped]**, which reduced accumulated dead-reckoning error without allowing isolated noisy measurements to cause large control changes.

### (d) No-Landmark Behavior

When no landmark was visible, the robot **[describe actual behavior]**. In our final implementation, this allowed the robot to continue for short intervals using its motion estimate rather than stopping immediately. **[If applicable: If no valid landmark was observed for ___ s / near a waypoint, the robot performed ___ to reacquire a tag.]**

This behavior was useful because the camera could not observe every landmark continuously along the full route.

---

## 3. Controller Performance

### (f) Path Smoothness and Correction Behavior

Figure 2 shows the estimated trajectory of the final run. The main corrections occurred **[where]**. Along the first segment, the robot **[slightly under-corrected / over-corrected / remained close to the reference]**, with corrections of approximately **[___]**. Near the second waypoint, **[describe behavior]**. On the return segment, **[describe behavior]**.

Overall, the path was **[smooth / moderately oscillatory / etc.]**. The trajectory shows **[whether corrections generally converged toward the reference or alternated across it]**. We intentionally used **[small/frequent or larger/infrequent]** corrections because **[brief reason]**. By the time the robot reached each waypoint, **[describe how much residual error remained and whether the controller made a final correction]**.

![Estimated trajectory](figures/trajectory.png)

<!--
Use the path itself to discuss:
- under- vs over-correction
- how much correction happens within each segment
- how much is left to correct near each waypoint
This directly follows the Piazza clarification for part (f).
-->

### (g) Total Distance Travelled

We estimated translational distance by summing the displacement of the robot center between successive logged pose estimates:

\[
D=\sum_k \sqrt{(x_{k+1}-x_k)^2+(y_{k+1}-y_k)^2}.
\]

The final estimated translational distance was **[___ m]**. We report translation of the robot center and exclude in-place rotation from this value.

<!-- Piazza clarification says rotation may be ignored. If useful, optionally add:
"The robot additionally accumulated approximately [___ rad] of in-place rotation."
-->

### (h) Waypoint Errors

Waypoint error was computed from the robot's onboard pose estimate when the controller declared each waypoint reached. Translation error is the Euclidean position difference from the target; rotation error is the wrapped yaw difference. The floor waypoint markers and final video provide a visual check that the robot physically reached the intended locations.

| Waypoint | Target pose \((x,y,\theta)\) | Estimated pose at arrival | Translation error | Rotation error |
|---|---|---|---:|---:|
| 1 | \((0,0,0)\) | [ ] | [ ] m | [ ] rad / [ ]° |
| 2 | \((1,0,0)\) | [ ] | [ ] m | [ ] rad / [ ]° |
| 3 | \((1,2,\pi)\) | [ ] | [ ] m | [ ] rad / [ ]° |
| 4 | \((0,0,0)\) | [ ] | [ ] m | [ ] rad / [ ]° |

### (i) Estimated 2D Trajectory

The controller logged its internal pose estimate throughout the run at approximately **[0.5 / 1.0] s** intervals. During periods with no visible AprilTag, the plotted pose came from the controller's dead-reckoning estimate; after valid landmark observations, the corrected pose was used. Figure 2 plots these estimates in the world frame together with the required waypoints.

<!-- Generate from logs, not manually from video. -->

### (j) Overall Performance

The final system **[briefly summarize success]**. Compared with open-loop motion, visual feedback was most useful for **[e.g. correcting heading drift after turns / reducing accumulated position error on long segments]**. The largest remaining source of error was **[camera pose noise / wheel slip / imperfect camera extrinsic / intermittent tag visibility / controller overshoot]**.

The main limitation of our implementation is **[___]**. With more time, we would improve **[one or two concrete items only: e.g. multi-tag fusion, camera extrinsic calibration, smoother continuous steering]**. Despite these limitations, the experiment demonstrated the intended closed-loop behavior: landmark observations were used to estimate the robot pose and correct motion errors while following the required waypoint sequence.

---

## Final Run Summary

- Final video: **[link]**
- Camera calibration file: **[path/name]**
- Number of AprilTags used: **[___]**
- Final route completion: **[successful / description]**
- Total translational distance: **[___ m]**
- Final endpoint translation error: **[___ m]**
- Final endpoint rotation error: **[___ rad / ___°]**
- Trajectory log: **[filename]**
