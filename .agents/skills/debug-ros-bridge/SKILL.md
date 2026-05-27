# Debug ROS Bridge

Use this skill when diagnosing ROS 2 bridge behavior.

## Workflow

1. Confirm the active scenario and ROS distro.
2. Check the graph:

```bash
ros2 topic list
ros2 node list
ros2 action list
```

3. Check timing and QoS-sensitive topics:

```bash
ros2 topic hz /clock
ros2 topic info /robot_001/state --verbose
ros2 topic info /robot_001/cmd_vel --verbose
```

4. Compare observed QoS with `configs/qos/default.yaml`.
5. Inspect `events.jsonl` for `COMMAND_REJECTED`, `SAFETY_STOP`, and bridge
   lifecycle events.
