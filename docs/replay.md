# Replay

Replay is a core feature, not an add-on.

Each run should emit:

- `scenario.yaml`
- `resolved_config.yaml`
- `events.jsonl`
- `metrics.json`
- `report.md`
- `rosbag/` when recording is enabled
- `traces/` when tracing is enabled

The initial replay command validates the artifact layout. Runtime-state replay
from event logs and rosbag playback will be added as the ROS 2 bridge matures.
