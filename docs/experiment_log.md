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

## 2026-11-13 - stationary range and view-angle survey

Seven additional stationary runs were summarized with
`tools/estimate_noise.py measurement`. The estimator centers samples separately
for each visible tag, then pools the within-tag residuals.

| Run | Visible IDs | Accepted samples | Forward std. (m) | Left std. (m) |
|---|---|---:|---:|---:|
| `r_tag0_d075_a00_01` | 0 | 46 | 0.000093 | 0.000024 |
| `r_tag0_d150_a00_01` | 0, 1 | 1080 | 0.004801 | 0.003628 |
| `r_tag0_d225_a00_01` | 0, 1, 7 | 706 | 0.003457 | 0.002017 |
| `r_tag0_d150_ap30_01` | 0, 1 | 292 | 0.001342 | 0.000571 |
| `r_tag0_d150_am30_01` | 0, 7 | 198 | 0.004222 | 0.003251 |
| `r_tag0_d150_ap45_01` | 1, 2 | 182 | 0.007608 | 0.004698 |
| `r_tag0_d150_am45_01` | 7 | 108 | 0.000201 | 0.000104 |

The maximum individual-tag standard deviation was approximately 0.00955 m
forward and 0.00630 m left for tag 2 in the nominal +45-degree run. The current
noise model, `sigma(r)=0.025+0.015r` m, is substantially larger than observed
random scatter. This is safe but may underweight the primary sensor relative to
command-only motion prediction.

The copied raw logs contained 4,487 observations, all accepted. Observation
rates varied from 1.37 to 11.17 Hz by run. The maximum recorded NIS under the
original model was 0.521. Offline replay through a fresh EKF accepted all 4,487
samples with `sigma(r)=0.010+0.005r`; NIS p95, p99, and maximum were 0.228,
0.489, and 3.858. An aggressive `0.005+0.003r` model rejected 275 samples and
was rejected as overconfident.

The configured measurement model is now `sigma(r)=0.010+0.005r` m. Exact
ground-truth map error will still capture systematic bias that cannot be inferred
from within-run scatter.

## 2026-11-13 - landmark-based motion pilot

Three single-primitive trials used tags 0 and 1, with 20 samples per tag before
and after motion and approximately 1.128 m landmark baseline.

| Trial | Command | Estimated motion | Yaw estimate | Fit RMSE | Bootstrap yaw std. |
|---|---|---|---|---:|---:|
| `straight_01` | 0.050 m | 0.05165 m forward, 0.00220 m left | -0.472 deg | 0.00219 m | 0.308 deg |
| `left_01` | +3.000 deg | -0.00103 m forward, -0.00093 m left | +1.549 deg | 0.00031 m | 0.051 deg |
| `right_01` | -3.000 deg | +0.00028 m forward, +0.00145 m left | -2.583 deg | 0.00003 m | 0.013 deg |

All signs and geometry checks passed. The rover traveled about 3.3% farther
than commanded in the straight pilot. More importantly, the same 0.10 s pulse
produced only about 52% of the nominal left increment but 86% of the nominal
right increment. No controller value is changed from one trial; collect 20
trials per primitive and calibrate left/right prediction separately. A later
30-sample requirement timed out under intermittent two-tag visibility before
motion and was rejected. The safe default remains 10 samples per tag over at
least 3 seconds; repeated trials reduce the borderline straight-yaw uncertainty.

A subsequent 10-sample attempt also timed out with empty successful-sample
counts. Because lowering the threshold cannot repair zero input, the node now
prints live `/detections` publisher/message counters, TF success/failure counts,
quality rejections, per-tag counts, and the last TF exception. A dedicated
`motion_calibration.launch.py` starts the complete support stack and calibration
in one command, removing the most likely missing-process failure mode. The
observation wait was extended to 60 seconds for genuinely slow but nonzero data.

## 2026-11-13 - completed motion calibration

Six accepted trials were collected for each primitive. Straight motion averaged
0.06830 m with 0.01096 m standard deviation for a nominal 0.05 m command. Left
pulses averaged +0.02020 rad with 0.01147 rad standard deviation; right pulses
averaged -0.02262 rad with 0.01725 rad standard deviation. This confirms severe
directional and trial-to-trial turn variation.

The motor commands remain unchanged. The closed-loop controller now predicts
translation with scale 1.3659, predicts left and right pulses separately, uses a
2-degree heading tolerance, requires two initialized landmarks before starting,
and permits only a bounded five-pulse zero-tag search during an existing turn.
Existing conservative process noise is kept because it exceeds the measured
motion scatter and lets landmark observations correct each stopped primitive.

## 2026-11-13 - 0.4 m square shakedown

The interrupted shakedown ran for about 194 seconds, reached three waypoints,
mapped seven tags, and accepted 300 of 301 observations. At the apparent stall,
the controller was turning at the fourth vertex to face the final edge. It was
still receiving and accepting tag 3 after nearly every pulse; the two-landmark
startup gate was not involved. The behavior looked stalled because calibrated
left pulses average only about 1.16 degrees, so a 90-degree corner requires many
stop-observe cycles. Turn-progress logging was added.

The runtime recovery policy is now explicit: one visible mapped landmark may
update the correlated EKF after startup; zero-tag translation causes an
indefinite hold; zero tags mid-turn permit at most five micro-search pulses
toward the existing target, with a stop/check after every pulse, followed by an
indefinite hold. No blind translation is permitted.
