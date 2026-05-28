# Add ROS Interface

Use this skill when adding or changing ROS 2 messages, services, actions, topics,
or QoS profiles.

## Workflow

1. Inspect `docs/interfaces.md`.
2. Add or edit files under `ros2_ws/src/genesis_nav_msgs/`.
3. Update bridge code under `genesis_nav/ros/` or `ros2_ws/src/genesis_nav_ros/`.
4. Update `docs/interfaces.md` in the same change.
5. Run:

```bash
colcon build --base-paths ros2_ws/src --packages-select genesis_nav_msgs genesis_nav_ros
colcon test --base-paths ros2_ws/src --packages-select genesis_nav_msgs genesis_nav_ros
```

If ROS 2 is not installed, state that clearly and run Python tests that still
cover the related runtime behavior.
