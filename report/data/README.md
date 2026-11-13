# Report data

Copy only the small, curated run files used to reproduce final report figures
into this directory. Keep bulk runtime logs and ROS bags outside Git.

For each final run, record:

- Git commit hash and configuration file.
- Shape, side length, lap count, and start pose.
- `slam_states.jsonl`, `observations.csv`, and controller event log.
- Ground-truth YAML used by the offline analysis.
- Generated metrics JSON.
