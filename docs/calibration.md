# HW3 process and measurement noise calibration

The values in `robot_control/config/hw3.yaml` are deliberately labeled
preliminary. Replace them from measurements on the HW3 low-carpet arena before
claiming quantitative results.

## Measurement noise R

For each test pose:

1. Keep the rover completely stationary.
2. Place one tag at a measured range and view angle, or use a mounted arena tag
   from a measured rover pose.
3. Start bringup with the controller disabled and a unique run name.
4. Collect at least 30 accepted observations after the camera pipeline settles.
5. Repeat near, medium, and far, including a moderately oblique view.

Each run produces `observations.csv`. Summarize one stationary run with:

```bash
python3 tools/estimate_noise.py measurement \
  ~/hw3_logs/r_near/observations.csv
```

Record the forward/left covariance and standard deviation. Fit a conservative
linear standard-deviation model

```text
sigma(range) = measurement_sigma_base_m
             + measurement_sigma_per_meter * range
```

that is at least as large as the observed spread across the tested ranges. Keep
the raw CSVs and fit plot as report evidence.

For the mounted tag 0, use this initial calibration matrix. Restart bringup after
every manual relocation and keep the controller disabled:

| Run name | Approximate base-link/tag range | Robot yaw relative to front-on |
|---|---:|---:|
| `r_tag0_d075_a00_01` | 0.75 m | 0 degrees |
| `r_tag0_d150_a00_01` | 1.50 m | 0 degrees |
| `r_tag0_d225_a00_01` | 2.25 m | 0 degrees |
| `r_tag0_d150_ap30_01` | 1.50 m | +30 degrees |
| `r_tag0_d150_am30_01` | 1.50 m | -30 degrees |
| `r_tag0_d150_ap45_01` | 1.50 m | +45 degrees |
| `r_tag0_d150_am45_01` | 1.50 m | -45 degrees |

The completed `stationary_tag0_01` run near 0.934 m supplies an additional
front-on checkpoint. Collect at least 60 seconds and 30 accepted observations
per run. Measure range from the `base_link` origin, record how yaw was measured,
and do not infer absolute accuracy from within-run scatter alone.

### Selected measurement model

The eight completed stationary runs contained 4,487 observations with no
quality or innovation rejection. The worst individual-tag standard deviation
was approximately 0.00955 m forward and 0.00630 m left. Replaying every raw
measurement through a fresh stationary EKF produced:

| Model | Accepted | Rejected | NIS p95 | NIS p99 | Maximum NIS |
|---|---:|---:|---:|---:|---:|
| `0.025+0.015r` | 4487 | 0 | 0.030 | 0.065 | 0.521 |
| `0.010+0.005r` | 4487 | 0 | 0.228 | 0.489 | 3.858 |
| `0.005+0.003r` | 4212 | 275 | 11.359 | 11.805 | 13.035 |

The selected model is therefore

```text
sigma(r) = 0.010 + 0.005 r meters.
```

It gives vision more weight than the original deliberately conservative model,
stays above measured random scatter, and preserves margin below the 2-D 99%
chi-square gate of 9.21. Revisit it only if controlled moving tests show that
unmodeled motion or systematic viewing bias causes plausible observations to be
rejected.

The replay is reproducible with:

```bash
python3 tools/replay_measurement_noise.py \
  runtime/logs/measurement_calibration/*/observations.csv \
  --sigma-base 0.010 \
  --sigma-per-meter 0.005
```

## Motion noise M (mapped to Q through G)

Use `report/data/motion_trials_template.csv` as the schema. For translation,
command one 5 cm primitive, measure the realized base-link displacement with
floor marks, reset the rover, and repeat at least 15 times. For rotation, repeat
the 0.10 s left and right pulses and measure yaw change with a large floor angle
reference. Use the same battery state and carpet as the final experiment.

Summarize the trials with:

```bash
python3 tools/estimate_noise.py motion report/data/motion_trials.csv
```

Use the mean error to decide whether the command calibration needs correction.
Use the sample covariance or a slightly conservative diagonal approximation for
the control-noise model. Report both the measurements and the final values;
do not present preliminary config values as calibrated results.

## Required sanity checks after tuning

- Prediction covariance grows during unobserved motion.
- A plausible landmark observation is normally accepted.
- A deliberately impossible observation is rejected by the NIS gate.
- Repeated observations reduce marginal covariance without making it collapse
  unrealistically to zero.
- Later-lap loop closure reduces uncertainty when the first tags reappear.
