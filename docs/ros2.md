# ROS 2 Integration

ROS 2 is the external contract for `genesis-nav`. Genesis API changes should not
force avoidable ROS graph or message contract changes.

Initial bridge scope:

- publish `/clock`
- publish per-agent state
- publish tf and tf_static
- subscribe per-agent `cmd_vel`
- expose navigation actions
- load QoS profiles from `configs/qos/`
- support rosbag profile driven recording

Lifecycle nodes are preferred for long-running bridge processes once resource
initialization and activation need to be explicit.
