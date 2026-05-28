# AGENTS.md

## Project Identity

genesis-nav is a ROS 2-native embodied runtime infrastructure for Genesis World.
It is not a Nav2 clone, RL framework, VLA framework, or demo-only wrapper.

## Architecture Rules

- Keep Genesis-specific code under `genesis_nav/genesis/`.
- Keep ROS 2-specific Python code under `genesis_nav/ros/`.
- Keep ROS 2 packages under `ros2_ws/src/`.
- Update `docs/interfaces.md` when changing public interfaces.
- Update `docs/experiments.md` when adding or changing benchmarks.
- Update `docs/decisions.md` for architectural tradeoffs.
- Prefer working code over abstractions.
- Do not introduce plugin systems without an issue explaining why.
- Keep the real robot path visible in runtime interfaces.
- Treat replay, observability, and metrics as core features.

## Test Commands

- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/unit`
- `colcon test --base-paths ros2_ws/src --packages-select genesis_nav_msgs genesis_nav_ros`
- `gnav run examples/scenarios/smoke.yaml --fast`

## Safety Rules

- Do not allow AI agents to publish actuator commands directly.
- All actuator commands must pass through `CommandGate`.
- Do not remove replay or logging paths.
- Command sources must include authority, requester, and timestamp metadata.
- AI agents may submit tasks, pause agents, and request stops. They may not
  disable safety gates or mutate active QoS/runtime config without an explicit
  allowlist.
