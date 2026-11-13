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

Use IDs 0 through 7 in counterclockwise wall order. Put two tags on each wall at
about one-third and two-thirds of the measured wall length. Mount every tag
upright, flat, and facing inward. A common center height near 0.20 m is a useful
starting point: the camera origin is about 0.115 m high and a 0.154 m tag centered
at 0.20 m has its lower edge near camera height.

For a nominal 3.302 m square, one-third is 1.101 m and two-thirds is 2.201 m:

| ID | Wall | Proposed tag center (m) | Inward face normal |
|---:|---|---|---|
| 0 | South | `(1.101, 0.000, 0.200)` | `+y` |
| 1 | South | `(2.201, 0.000, 0.200)` | `+y` |
| 2 | East | `(3.302, 1.101, 0.200)` | `-x` |
| 3 | East | `(3.302, 2.201, 0.200)` | `-x` |
| 4 | North | `(2.201, 3.302, 0.200)` | `-y` |
| 5 | North | `(1.101, 3.302, 0.200)` | `-y` |
| 6 | West | `(0.000, 2.201, 0.200)` | `+x` |
| 7 | West | `(0.000, 1.101, 0.200)` | `+x` |

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
6. Put removable floor marks at the robot start pose and intended path vertices.
7. Record the start `base_link` center and yaw in the same world frame.

## Visibility survey before autonomous motion

Carry or manually place the stationary rover at expected path vertices and edge
midpoints. Record tag ID, range, decision margin, and detection rate while facing
the next wall. Adjust tag height or along-wall position only if the survey shows
a repeatable blind region. Any departure from roughly even spacing should be
justified in the report as improving mapping/localization visibility.

Start with a conservative square side near 1.8 m, centered in the room, and an
octagon that fits within a similar footprint. These are test values, not final
report values. Increase them only after confirming clearance and tag visibility.

With the nominal 3.302 m room and the controller's local-path convention, a
centered 1.8 m square starts near world pose `(0.751, 0.751, 0)`. The proposed
0.75 m octagon spans approximately 1.811 m by 1.811 m; a centered placement
starts near `(1.276, 0.746, 0)` because the later octagon vertices extend about
0.530 m to the left of its initial local-frame x coordinate. Mark and measure
these poses rather than relying on the nominal calculations.
