# Replay

Replay is a core feature, not an add-on.

Each run should emit:

- `scenario.yaml`
- `resolved_config.yaml`
- `events.jsonl`
- `metrics.json`
- `report.md`
- `rosbag/` when recording is enabled (`--record --ros` writes a rosbag2 bag;
  `--record` alone leaves a `RECORDING_SKIPPED` marker)
- `traces/` when tracing is enabled

`gnav run --record --ros` mirrors bridged topics into `run_dir/rosbag/` using
the profile in `configs/rosbag/minimal.yaml` (override with
`--rosbag-profile`). The bag uses rosbag2 sqlite3 storage and can be played
back with standard ROS 2 tooling once the workspace is sourced.

`gnav replay <run_dir>` validates this layout. With `--to-rosbag` it
re-simulates the stored scenario through the ROS bridge and writes
`run_dir/rosbag/` (requires rclpy, rosbag2_py, and genesis_nav_msgs). This
replaces a prior `RECORDING_SKIPPED` marker from runs that used `--record`
without `--ros`. Override the topic list with `--rosbag-profile`.

The initial replay command validates the artifact layout. Runtime-state replay
from event logs and rosbag playback will be added as the ROS 2 bridge matures.
