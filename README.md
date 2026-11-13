# CSE 276A Homework 3 - EKF SLAM

This repository contains the Rubik Pi ROS 2 stack for CSE 276A Homework 3.
The HW3 system builds an initially unknown landmark map while estimating the
rover pose, then uses that estimate to drive closed-loop square and octagon
trajectories.

The preserved HW2 implementation is available at Git tag `hw2-final`. The
active branch is intentionally organized around HW3 only.

## Repository map

- `robot_vision/`: Rubik Pi camera driver and calibrated camera parameters.
- `apriltag_ros/`: AprilTag detector used for landmark observations.
- `robot_control/`: motor bridge, EKF-SLAM, experiment controller, launch, and
  configuration, including a supervised one-primitive calibration command.
- `tools/`: offline validation, plotting, and landmark-error analysis.
- `report/`: report source, experiment-data instructions, and figures.
- `docs/`: requirement traceability, arena setup, and runtime instructions.
- `hw3-2026.pdf`: official assignment handout.

## Safety and experimental policy

The rover has no wheel encoders and its IMU is not trusted. Motion prediction
therefore uses calibrated commanded increments with deliberately nonzero
process noise. The default experiment policy moves about 5 cm, stops, waits for
a fresh AprilTag update, and only then decides the next action.

Ground-truth landmark coordinates are evaluation-only. They must never be read
by the online SLAM node.

See `docs/runtime.md` for deployment and commands. See
`docs/hw3_requirements.md` for the handout-to-report checklist.

## Offline verification

The estimator and path geometry can be tested without ROS or the rover:

```bash
PYTHONPATH=robot_control python3 -m unittest discover \
  -s robot_control/test -v
```

Final-run plots and error metrics are generated with
`tools/analyze_hw3_run.py`. Analysis-only Python requirements are listed in
`tools/requirements-analysis.txt`.
