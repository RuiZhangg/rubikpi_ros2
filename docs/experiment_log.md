# HW3 experiment log

This file records diagnostic conclusions and evidence that should survive
between arena sessions. Final numerical claims still require the corresponding
raw logs under `report/data/`.

## 2026-11-13 - stationary tag 0 pipeline check

### Setup

- Rover stationary and approximately front-on to tag 0.
- Tag family `tag36h11`, configured detection-edge size 0.154 m.
- Controller disabled; no manual movement during the EKF run.
- Run name `stationary_tag0_01`.

### Observations

- A raw-camera-only test reported an average rate around 2.8 Hz at 1280 by 720.
- A separate rectification test reported an average rate around 4.7 Hz. Because
  these were separate sampling windows, the two rates are not an end-to-end
  throughput comparison.
- Tag 0 detections had hamming 0 and decision margin about 174-175.
- Observed 3-D tag range was approximately 0.934 m.
- Robot-frame planar measurement was approximately 0.907 m forward and
  -0.0363 m left.
- The first row initialized tag 0. All supplied subsequent rows were accepted,
  with very small NIS values relative to the 9.21 gate.

### Conclusion

Camera capture, rectification, tag detection, timestamped TF lookup, EKF
initialization, repeated correction, and logging all passed this stationary
checkpoint. Earlier large NIS values occurred after the rover was moved by hand
without a control increment; they are not evidence that the innovation gate is
too strict. No gate or noise parameter is changed from this checkpoint.

### Next evidence

Collect stationary runs at multiple ranges and view angles, restarting the
SLAM process after each manual relocation. Use within-run scatter to estimate
random measurement noise and tape-measured geometry to identify range/lateral
bias. Preserve unique run names and copy the selected CSV and JSONL files into
`report/data/` before final analysis.
