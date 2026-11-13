# Robot deployment and runtime

## Copy and build

From the development computer, copy or synchronize the repository into
`~/ros2_ws/src/rubikpi_ros2` on the rover. On the rover:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

### If the build asks for `robot_teleop_launch.py`

That filename exists only in the old HW2 `setup.py`. First verify the rover
source instead of repeatedly rebuilding stale metadata:

```bash
cd ~/ros2_ws/src/rubikpi_ros2
git status --short --branch
git log -1 --oneline
grep -n 'robot_teleop_launch' robot_control/setup.py
```

With the HW3 source, the `grep` command prints nothing and `setup.py` contains
`glob('launch/*.py')`. Update the source, then remove only the stale
`robot_control` build/install products:

```bash
cd ~/ros2_ws/src/rubikpi_ros2
git fetch origin
git checkout main
git pull --ff-only

cd ~/ros2_ws
rm -rf build/robot_control install/robot_control
colcon build --symlink-install --allow-overriding apriltag_ros
source ~/ros2_ws/install/setup.bash
```

The `apriltag_ros` override warning is expected because this workspace carries
the course-compatible detector configuration over the Jazzy underlay package.
The missing launch-file error is not caused by that warning.

The package now installs working entry points for `motor_control`,
`velocity_mapping`, `hw3_slam`, and `hw3_controller`; direct `python3` paths are
no longer required.

The bringup launch reads the fixed camera transform from
`robot_control/config/camera_extrinsics.yaml`. A different arena does not change
this calibration. Recalibrate it only if the camera moved relative to the rover.

## Safe sensing and SLAM check

Start the complete perception, motor bridge, and empty-map SLAM stack without
motion:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch robot_control hw3_bringup.launch.py \
  run_controller:=false \
  run_name:=visibility_check
```

Confirm:

```bash
ros2 topic hz /camera/image_rect
ros2 topic echo /detections --once
ros2 topic echo /hw3/slam_map --once
ros2 topic echo /hw3/slam_pose --once
```

The first observed tag should initialize a landmark. The online node must not
load `ground_truth.yaml`.

## Autonomous square run

Keep one person next to the rover with Ctrl-C ready. Use a unique run name:

```bash
ros2 launch robot_control hw3_bringup.launch.py \
  run_controller:=true \
  shape:=square \
  side_length_m:=1.8 \
  laps:=1 \
  run_name:=square_1lap_01
```

## Autonomous octagon run

```bash
ros2 launch robot_control hw3_bringup.launch.py \
  run_controller:=true \
  shape:=octagon \
  side_length_m:=0.75 \
  laps:=1 \
  run_name:=octagon_1lap_01
```

The side lengths above are initial test values. Replace them after the visibility
and clearance survey. Repeat with `laps:=2` or more for the required multi-lap
results.

## Expected behavior

- The controller does not move until at least one tag has initialized the map.
- Translation occurs in approximately 5 cm increments.
- Turns use the successful HW2 settings: angular command 0.002 for 0.10 s, with
  a predicted increment near 3 degrees.
- After every primitive the rover stops and waits for a new accepted tag update.
- If a tag leaves view during a commanded turn, the controller may continue
  calibrated turn pulses toward that same target yaw to search for the next tag.
  Blind translation is never allowed; it holds still if no update arrives after
  translation or after reaching the intended turn heading.
- Logs are written under `~/hw3_logs/<run_name>/`.

## Recommended staged tests

1. Perception and TF while stationary.
2. Empty-map initialization with each of the eight IDs.
3. One 5 cm translation while restrained and supervised.
4. One left and one right turn pulse.
5. Several 5 cm steps toward one tag.
6. One short path edge.
7. One full square, then one full octagon.
8. Multi-lap runs only after single-lap logs and plots are plausible.
