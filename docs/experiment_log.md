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

No parameter is changed yet. The pasted summaries contain accepted samples only
and do not provide rejection rates, and nominal run names are not sufficiently
precise ground truth for estimating systematic bias. Preserve the eight raw
observation CSVs so acceptance rate, NIS distribution, actual range, and bias
can be audited before selecting the final model. A provisional moving-test
candidate is `sigma(r)=0.010+0.005r` m, which remains above the observed random
scatter across the tested range.
