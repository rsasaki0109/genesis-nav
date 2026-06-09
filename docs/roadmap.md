# Roadmap

## Phase 1: Genesis-Native Runtime

- single-host runtime
- multi-agent scenarios
- ROS 2 bridge
- rosbag and event replay foundations
- basic navigation
- basic fleet dispatch

## Phase 2: ROS 2 Robot Deployment

- real robot adapter
- Nav2 adapter
- teleop adapter
- hardware diagnostics
- safety gate
- sim-real config parity

## Phase 3: Fleet and Distributed Runtime

- multi-host runtime
- resource reservation
- distributed task allocation
- Open-RMF adapter
- cloud-side monitor
- network-aware QoS profiles

## Phase 4: Embodied AI Integration

- semantic map
- VLM scene query
- task planning
- embodied memory
- AI-agent tool API
- human-in-the-loop supervision

## Phase 5: Multi-Simulator and Real-World Benchmark

- Genesis primary
- Gazebo adapter
- Isaac adapter
- Webots and CoppeliaSim comparison
- real robot replay
- sim-real gap reports

## v0.1 MVP

A user can install `genesis-nav`, run one command, watch multiple robots navigate
in Genesis, inspect ROS 2 topics, record a rosbag, replay the run, and see
metrics.

## v0.2 status

Phase 2 items (real robot, Nav2, teleop, diagnostics, safety gate) shipped in
v0.2.0 along with real Genesis 1.0, spatial safety (detect → yield → head-on
reroute), and a furnished warehouse world. See `docs/roadmap_v02.md` for the
v0.1 → v0.2 boundary and remaining follow-ups.
