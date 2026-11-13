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

Do not diagnose the complete stack until each perception stage below works.
Keep the rover and camera stationary throughout one EKF process. If the rover is
carried or manually rotated, restart the SLAM node: manual motion does not
publish a `/hw3/control_increment`, so the filter correctly treats subsequent
measurements as inconsistent.

### 1. Stop stale camera processes

After pressing Ctrl-C on an earlier launch, wait for every process to exit and
check that no earlier camera pipeline remains:

```bash
ros2 node list
ps -ef | grep -E '[r]obot_vision_camera_node|[r]ectify_node|[a]priltag_node|[h]w3_slam'
```

If one of those processes remains after its launch terminal has exited, stop
only the stale process before continuing. Two camera nodes can contend for the
Rubik Pi camera service.

### 2. Test raw camera images

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_vision_camera robot_vision_camera.launch.py
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
timeout 30 ros2 topic hz /camera/image_raw
ros2 topic echo /camera/camera_info --once --field header
```

Do not continue unless `/camera/image_raw` has a nonzero rate. A warning showing
zero received images is a camera/driver problem, not an AprilTag or EKF problem.
First stop duplicate camera processes and retry. If the raw rate remains zero,
reboot the rover, reconnect over SSH, source both workspaces, and repeat this
raw-camera test before starting any other nodes.

### 3. Test rectification

Leave the camera running. In Terminal 3:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run image_proc rectify_node --ros-args \
  -r image:=/camera/image_raw \
  -r camera_info:=/camera/camera_info \
  -r image_rect:=/camera/image_rect
```

In Terminal 2:

```bash
timeout 30 ros2 topic hz /camera/image_rect
```

Do not continue unless rectified images have a nonzero rate. Unequal image and
CameraInfo counts are acceptable when every received image becomes a
synchronized pair. For example, 2 images, 10 CameraInfo messages, and 2 pairs
means no image was lost to synchronization; the effective image rate is merely
slow.

### 4. Test AprilTag detection

With tag 0 centered in view, start the detector in Terminal 4:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:=/camera/image_rect \
  -r camera_info:=/camera/camera_info \
  --params-file ~/ros2_ws/src/rubikpi_ros2/apriltag_ros/cfg/tags_36h11.yaml
```

In Terminal 2, allow at least 60 seconds because processed vision is slow:

```bash
timeout 60 ros2 topic echo /detections
```

Confirm that the returned ID is 0, the family is `36h11`, hamming is no more
than 1, and decision margin is at least 20. Stop all four terminals before the
complete bringup so that a second camera node is not created.

### 5. Test stationary EKF initialization

Place the rover in front of tag 0 before launching, then do not touch it. Use a
new run name every time:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch robot_control hw3_bringup.launch.py \
  run_controller:=false \
  run_name:=stationary_tag0_01
```

In another sourced terminal, check every stage:

```bash
timeout 30 ros2 topic hz /camera/image_raw
timeout 30 ros2 topic hz /camera/image_rect
timeout 60 ros2 topic echo /detections
ros2 topic echo /hw3/slam_map --once
ros2 topic echo /hw3/slam_pose --once
```

The first observed tag should initialize a landmark. The online node must not
load `ground_truth.yaml`. Leave this test stationary for at least 60 seconds so
several slow observations can arrive. One initialization followed by accepted
updates is the expected result. Do not loosen the NIS gate merely to accept
stationary measurements that jump.

The SLAM node creates these files immediately at startup, even if there are no
detections:

```bash
ls -lah ~/hw3_logs/stationary_tag0_01
wc -l ~/hw3_logs/stationary_tag0_01/observations.csv \
  ~/hw3_logs/stationary_tag0_01/slam_states.jsonl
tail -n 20 ~/hw3_logs/stationary_tag0_01/observations.csv
```

Reusing a run name reuses its directory and opens both files with truncation;
it does not create a second directory. Always use a unique run name for evidence
that may be needed in the report.

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
