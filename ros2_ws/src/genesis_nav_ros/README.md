# genesis_nav_ros

Marker package for the genesis-nav ROS 2 bridge.

The bridge implementation lives in the Python package `genesis_nav.ros` and is
launched in-process by `gnav run --ros`. This colcon package exists so
downstream packages (such as `genesis_nav_bringup`) can declare a build/exec
dependency on the bridge boundary.

## Usage

```bash
gnav run examples/scenarios/smoke.yaml --fast --ros \
  --qos-profile configs/qos/default.yaml
```

The bridge publishes:

- `/clock` (rosgraph_msgs/Clock)
- `/genesis_nav/events` (genesis_nav_msgs/RuntimeEvent)
- `/genesis_nav/scenario_state` (genesis_nav_msgs/ScenarioState)
- `/genesis_nav/fleet_state` (genesis_nav_msgs/FleetState)
- `/<agent>/state` (genesis_nav_msgs/AgentState)
- `/<agent>/odom` (nav_msgs/Odometry)
- `/tf`, `/tf_static`

The bridge subscribes to `/<agent>/cmd_vel` and forwards each Twist through
`CommandGate` as a `TELEOP`-authority `RuntimeCommand` before any actuator
applies it.
