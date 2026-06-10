# v0.1 → v0.2 Boundary

One-page summary of what v0.1 shipped, what v0.2 adds, and what remains open.

## v0.1 delivered

- Genesis-hosted (fallback integrator) multi-agent runtime with YAML scenarios
- Grid / straight-line planners, behavior state machine, fleet dispatcher
- ROS 2 bridge (`/clock`, per-agent `/state` / `/odom`, tf, `/genesis_nav/events`)
- `CommandGate` safety boundary + `AgentToolApi`
- Replay artifacts (`events.jsonl`, `metrics.json`, `env.json`, `report.md`)
- `gnav bench` regression harness with expectation predicates

## v0.2 adds (closed in 0.2.0)

| Area | What shipped |
|------|----------------|
| Real robot | `ros2_robot` backend, loopback transport, command-watchdog auto-poll |
| Nav2 | Planner + controller delegation, both through `CommandGate`; integration benchmarks gated behind `--include-integration` |
| Teleop | `submit_teleop_command`, bridge unified onto runtime hold window |
| Diagnostics | Per-agent health read-model + periodic events + ROS `DiagnosticArray` |
| Dynamic obstacles | Grid deltas, replan on blocked path |
| Spatial safety | Detect → near-miss → yield → diagnostics; **head-on lateral reroute**; **costmap reservation** |
| Genesis | Real Genesis 1.0 API, kinematic diff-drive, URDF wheel-joint path, furnished warehouse world |
| Observability (0.2.2) | Rosbag record/export, `doctor --json`, `BENCHMARK_REPORT`, Wilson `success_rate_ci` |
| Navigation (0.2.2) | Grid `inflate_cells`, task `dwell_sec`, `type: holonomic` adapter |

## Still open (v0.3+ direction)

- Smarter priority (goal distance, RVO)
- Nav2 costmap layers / recovery behaviors
- Non-loopback real-hardware transports
- Multi-host / distributed runtime, Open-RMF adapter
- Phase 4 embodied-AI tools (semantic map, VLM query, task planning)

## How to choose a backend

```bash
gnav run examples/scenarios/smoke.yaml --fast                    # fallback (default)
gnav run examples/scenarios/smoke.yaml --fast --backend genesis  # real Genesis
gnav run examples/scenarios/real_robot_loopback.yaml --backend ros2_robot
```

Set `GENESIS_NAV_SPAWN_URDF=1` to spawn the bundled diff-drive URDF on Genesis
instead of a kinematic box.

See also: `docs/roadmap.md` (phases), `PLAN.md` §20 (v0.2 direction),
`docs/decisions.md` (ADRs).
