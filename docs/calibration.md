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
