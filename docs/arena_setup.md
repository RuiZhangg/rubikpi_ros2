# HW3 arena and AprilTag placement

## Coordinate convention

Measure all four walls; do not assume the room is perfectly square. A convenient
ground-truth frame uses the southwest corner as `(0, 0)`, positive x toward the
east wall, and positive y toward the north wall. Mark that origin physically so
it can be reconstructed between sessions.

The currently reported side length is 10 ft 10 in, approximately 3.302 m. The
table below is a placement proposal, not ground truth. Replace every proposed
number with a tape-measured tag-center coordinate after mounting.

## Recommended placement

The mounted IDs run counterclockwise starting on the north wall: tag 0 is the
right/east tag, tag 1 is the left/west tag, tags 2 and 3 are the north and south
tags on the west wall, tags 4 and 5 continue across the south wall, and tags 6
and 7 continue up the east wall. All tags are upright, at the wall plane, and
face inward. Tags 4 and 5 have measured center height 0.124 m; the other tags
have measured center height 0.210 m.

For a nominal 3.302 m square, one-third is 1.101 m and two-thirds is 2.201 m:

| ID | Wall/location | Nominal center (m) | Measured height | Inward normal |
|---:|---|---|---:|---|
| 0 | North, east/right | `(2.201, 3.302)` | 0.210 m | `-y` |
| 1 | North, west/left | `(1.101, 3.302)` | 0.210 m | `-y` |
| 2 | West, north/upper | `(0.000, 2.201)` | 0.210 m | `+x` |
| 3 | West, south/lower | `(0.000, 1.101)` | 0.210 m | `+x` |
| 4 | South, west/left | `(1.101, 0.000)` | 0.124 m | `+y` |
| 5 | South, east/right | `(2.201, 0.000)` | 0.124 m | `+y` |
| 6 | East, south/lower | `(3.302, 1.101)` | 0.210 m | `-x` |
| 7 | East, north/upper | `(3.302, 2.201)` | 0.210 m | `-x` |

The x/y values remain nominal until the mounted centers and four wall lengths
are tape-measured. Do not use them for final landmark-error calculations yet.

This spacing keeps tags away from corner occlusion and distributes observations
along each wall. When the rover drives a wall-aligned square, its forward camera
primarily observes tags on the wall ahead. Do not finalize path dimensions until
the range/angle test confirms that at least one tag remains detectable through
each approach and turn.

## Mounting and measurement checklist

1. Print all tags at the same scale and confirm the black detection-edge length
   is 0.154 m, not merely the paper size.
2. Mount tags on rigid, matte backing. Avoid curled paper, glare, and carpet-level
   obstructions.
3. Keep the printed square vertical. Measure the center, not an outer paper edge.
4. Record tag ID, center x/y/z, wall, and face direction in
   `robot_control/config/ground_truth.yaml`.
5. Photograph each wall and the tape-measure reference.
6. Mark the robot's start `base_link` center and yaw for repeatable trials and
   the required offline SLAM-to-world transform.
7. Treat any other path-vertex marks as optional evaluation references. The
   online controller and estimator must not observe or use them.

## Visibility survey before autonomous motion

Carry or manually place the stationary rover at expected path vertices and edge
midpoints. Restart the SLAM process after every manual displacement, or perform
this survey using detector/TF output without the EKF. Manual movement is not a
control input and will otherwise cause valid measurements to fail the innovation
gate. Record tag ID, range, decision margin, and detection rate while facing the
next wall. Adjust tag height or along-wall position only if the survey shows a
repeatable blind region. Any departure from roughly even spacing should be
justified in the report as improving mapping/localization visibility.

Start with a conservative square side near 1.8 m, centered in the room, and an
octagon that fits within a similar footprint. These are test values, not final
report values. Increase them only after confirming clearance and tag visibility.

## Optional floor references for evaluation

The assignment does not require marked path vertices, and the rover must follow
the square and octagon from its estimated SLAM pose rather than floor marks. A
start-pose mark is useful because the measured start pose is needed for the
offline world-frame transform. The remaining coordinates below are optional
human references for repeatability and independent path-error checks.

### Minimal recommended start marks

Use two composite tape marks if the square and octagon remain centered: one for
each trajectory. Each composite mark is a cross under the `base_link` origin
plus a 0.30 m straight heading line pointing east. Equivalently, four small dots
can be used: a center and forward alignment point for each path.

For measured arena width `W`, height `H`, square side `n`, and octagon side `m`:

- Square center mark: `S-C=((W-n)/2, (H-n)/2)`.
- Square forward point: `S-F=S-C+(0.30, 0)`.
- Let `B=m(1+sqrt(2))` be the octagon footprint.
- Octagon center mark: `O-C=(W/2-m/2, (H-B)/2)`.
- Octagon forward point: `O-F=O-C+(0.30, 0)`.

With the nominal `W=H=3.302 m`, `n=1.8 m`, and `m=0.75 m`, these are
`S-C=(0.751,0.751)`, `S-F=(1.051,0.751)`, `O-C=(1.276,0.746)`, and
`O-F=(1.576,0.746)` m. Recompute them from the measured wall dimensions before
placing permanent references.

If optional references are used, mark the `base_link` center, not the front
bumper or camera. The
coordinates below use the nominal 3.302 m square, the southwest origin, and a
starting yaw of 0 radians (east). Recompute them from the measured east-west
width `W` and north-south height `H` if either differs materially from 3.302 m.

For the centered 1.8 m square, use
`x_low=(W-1.8)/2`, `x_high=x_low+1.8`, and the corresponding y values. Nominal
marks, visited counterclockwise, are:

| Mark | World coordinate (m) | Departing direction |
|---|---|---|
| S0 | `(0.751, 0.751)` | east; robot start |
| S1 | `(2.551, 0.751)` | north |
| S2 | `(2.551, 2.551)` | west |
| S3 | `(0.751, 2.551)` | south, then return to S0 |

For the centered regular octagon with side length 0.75 m, the footprint is
`0.75(1+sqrt(2)) = 1.811 m` square. Nominal marks are:

| Mark | World coordinate (m) | Departing direction |
|---|---|---|
| O0 | `(1.2760, 0.7457)` | east; robot start |
| O1 | `(2.0260, 0.7457)` | northeast, 45 degrees |
| O2 | `(2.5563, 1.2760)` | north, 90 degrees |
| O3 | `(2.5563, 2.0260)` | northwest, 135 degrees |
| O4 | `(2.0260, 2.5563)` | west, 180 degrees |
| O5 | `(1.2760, 2.5563)` | southwest, -135 degrees |
| O6 | `(0.7457, 2.0260)` | south, -90 degrees |
| O7 | `(0.7457, 1.2760)` | southeast, -45 degrees, then O0 |

These 4 square points plus 8 octagon points are optional evaluation locations,
not online navigation inputs. Mark S0/O0 separately if both trials use different
starts. Record any coordinates used for evaluation rather than silently treating
these nominal calculations as measurements.
