# HW3 requirement traceability

This checklist is derived from `hw3-2026.pdf` and the supplied instructor
clarifications. It exists to prevent a report or experiment deliverable from
being missed. Check a box only after evidence has been collected.

## Environment and ground truth

- [ ] Use a square environment near 10 ft by 10 ft and no smaller than 8 ft by
  8 ft.
- [ ] Place two landmarks on each of the four sides.
- [ ] Record tag IDs, tag-center coordinates, height, and wall dimensions.
- [ ] Provide a measured top-down ground-truth sketch in the report.
- [ ] Record the robot's measured start pose for the offline SLAM-to-world
  transform. Do not provide this pose to the online filter.

## Online algorithm

- [x] Use a Kalman-filter-based SLAM system; an EKF is the selected design.
- [x] Define state vector `x` and measurement vector `z`.
- [x] Derive and report system Jacobian `F`, control Jacobian `G`, and
  measurement Jacobian `H`.
- [x] Define initial state and covariance.
- [ ] Explain how process noise `Q` and measurement noise `R` were measured.
- [x] Dynamically add a tag on its first observation. The filter must not
  preallocate or assume eight landmarks.
- [x] Explain behavior when a landmark leaves the field of view.
- [x] Explain behavior when a previously mapped landmark reappears.
- [x] Use the current SLAM pose to correct trajectory drift.

## Required experiments

- [ ] One-lap square trajectory.
- [ ] Multiple-lap square trajectory.
- [ ] One-lap regular 8-edge octagon trajectory.
- [ ] Multiple-lap octagon trajectory.
- [ ] Select and justify square side length `n`.
- [ ] Select and justify octagon side length `m`.
- [ ] Return close to the starting position.
- [ ] Collect sufficiently dense observations; the selected default is a
  stop-and-observe step of about 5 cm.

## Required results

- [ ] Plot each trajectory from the algorithm estimate, not from visual
  inspection.
- [ ] Plot the estimated landmarks with position covariance ellipses.
- [ ] Transform the estimated map from the arbitrary SLAM frame into the
  measured world frame.
- [ ] Compute average landmark-position error against ground truth.
- [ ] Compare square and octagon maps.
- [ ] Compare one lap with multiple laps and discuss whether the map improves.
- [ ] Discuss calibration, motion, visibility, and landmark-placement
  tradeoffs.

## Submission

- [ ] Report answers every item above.
- [ ] Report contains an unlisted YouTube link showing one square lap and one
  octagon lap.
- [ ] Code ZIP is prepared and independently checked.
