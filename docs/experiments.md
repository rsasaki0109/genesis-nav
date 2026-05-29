# Experiments

## Current Baseline

| Date | Scenario | Commit | Result | Notes |
|---|---|---|---|---|
| 2026-05-28 | `examples/scenarios/smoke.yaml` | pending | success_rate=1.0, time_to_goal≈5.3s, path≈2.14m, 265 sim steps | Single diff-drive agent navigates to `(2, 1, 0)` via in-memory kinematics fallback. |
| 2026-05-28 | `examples/scenarios/warehouse_10_agents.yaml` | pending | success_rate=1.0 over 3 tasks, time_to_goal_mean≈11.4s, 568 sim steps | 10 spawned diff-drive agents with 3 assigned tasks; no collision modelling yet. |
| 2026-05-28 | `examples/scenarios/smoke.yaml --ros` | pending | success_rate=1.0, time_to_goal≈5.3s, 265 sim steps, bridge publishes 6+N topics | ROS 2 bridge enabled; `/clock`, `/genesis_nav/events`, per-agent `/state` and `/odom`, tf, tf_static visible while events flow to JSONL and topic. |
| 2026-05-28 | `examples/scenarios/smoke.yaml --backend genesis` | pending | aborts with install hint when Genesis is missing | Confirms `gnav doctor` and the CLI surface report Genesis availability before runtime startup. Run on a host with Genesis to record the in-Genesis baseline. |
| 2026-05-28 | `examples/scenarios/warehouse_10_agents.yaml --fast` (post-dispatcher) | pending | success_rate=1.0 over 3 tasks via dispatcher, task_dispatched_count=3, task_pending_peak=3, 568 sim steps | All tasks flow through `submit_task → dispatcher → assign_task`; reservation counters stay 0 because the scenario declares no shared resources. |
| 2026-05-28 | `examples/scenarios/smoke.yaml --fast` (post-J: env+replay strict) | pending | env.json populated (python/platform/git/ros_distro/genesis_version), replay strict ok with `--print-events` streaming SCENARIO_STARTED→TASK_ASSIGNED→TASK_STARTED→TASK_SUCCEEDED→SCENARIO_FINISHED | Confirms Workstream J run-directory contract: env metadata captured, replay refuses missing artifacts / corrupt events / metrics without required keys. |
| 2026-05-28 | `gnav bench --run benchmarks/{nav_basic,multi_agent,runtime,humanoid}` | pending | 4 suites, 1 scenario each, all green: nav_basic 1/1 (sr=1.0, ttg≈5.3s), multi_agent 1/1 (4 succeeded), runtime 1/1 (sim_steps=265), humanoid 1/1 | First Workstream K harness pass. Reports under `benchmarks/_runs/<suite>_report.json` each list `run_dir` for failure debug. |
| 2026-05-28 | `examples/scenarios/smoke.yaml --fast` (post-H: navigation MVP) | pending | success_rate=1.0, BEHAVIOR_STATE_CHANGED stream idle→assigned→planning→executing→succeeded→idle, PLAN_RESOLVED carries planner+waypoint_count, stuck/recovery counters present in metrics.json | Confirms Workstream H wiring: `StraightLinePlanner` fallback, behavior state machine emitted alongside task lifecycle, no regressions in the 94 pre-H tests. |
| 2026-05-28 | Stuck-and-recover unit test (`tests/unit/test_navigation_mvp.py::test_stuck_detector_triggers_recovery_then_fails`) | pending | AGENT_STUCK fires once, behavior transitions executing→recovering→executing→failed→idle, TASK_FAILED reason="stuck" | First end-to-end exercise of the new recovery loop. `stuck_window_sec=0.5`, `max_recovery_retries=1`. |
| 2026-05-28 | Workstream N — docs and launch surface (no runtime change) | pending | CONTRIBUTING.md + 5 new issue templates + good-first-issues catalogue + comparison table + demo / launch scripts shipped; no rerun of smoke required | `gnav run` and `gnav bench` outputs unchanged; verified by inspection that no `genesis_nav/`, `ros2_ws/`, or `tests/` source files were touched. |
| 2026-05-29 | `make demo-gif` (asciinema + agg) | pending | `docs/media/smoke_demo.gif` 486KB / 979x649, cast 11KB; sequence: `gnav run` success_rate=1.0 → `report.md` head → `gnav replay --print-events` 5 events → `gnav bench` 1/1 passed | First fully scripted README hero capture; `make demo-gif` reproduces deterministically from `scripts/record_demo.sh`. |
| 2026-05-29 | v0.1.0 release tag | pending | All 3 CI workflows (`ci` / `docs` / `ros2`) green on `efde985`; pyproject bumped 0.1.0a0 → 0.1.0; CHANGELOG.md added | Closes PLAN.md §9 v0.1 milestone. Outstanding items rolled into v0.2 (see issue #10). |
| 2026-05-29 | `examples/scenarios/smoke.yaml --fast --backend ros2_robot` | pending | first v0.2 slice: `--backend ros2_robot` builds a real rclpy node, publishes `/<agent>/cmd_vel`, subscribes `/<agent>/odom`, exits 0; 116 unit tests pass (+7 ros2_robot, all rclpy-free via `FakeRobotTransport`) | Dry connection only — no odom source on the host, so poses stay at origin. Verifies the outbound hardware edge + `CommandGate` path; real-robot loop closure is future work. See the 2026-05-29 real-robot ADR. |
| 2026-05-29 | dynamic-obstacle replan (unit e2e, `test_dynamic_obstacles.py`) | pending | second v0.2 slice: obstacle dropped on a 6-wide grid corridor at `at_sec=0.1` → `OBSTACLE_CHANGED` recorded, agent re-enters `planning` (`REPLAN_TRIGGERED`), routes around `[3,1]`, reaches goal; success_rate=1.0, replan_count≥1; 123 unit tests pass (+7) | Replan extends the planner contract (`OccupancyGrid.with_blocked`, `executing→planning` edge), not a new subsystem. Deltas are events so replays reconstruct the timeline. See the 2026-05-29 dynamic-obstacles ADR. |
| 2026-05-29 | Nav2 planner backend (unit, `test_nav2_planner.py`) | pending | third v0.2 slice: `runtime.navigation.planner: auto\|grid\|straight\|nav2` selector + `Nav2Planner` delegating to a `Nav2PathService`; selection + delegation verified with `FakeNav2PathService` (rclpy/nav2_msgs mocked out); 133 unit tests pass (+10) | Nav2 stays a planner backend behind `plan()`, not a runtime replacement. Generalizes issue #9's selector. Global-planning delegation only; controller `cmd_vel`-through-`CommandGate` + `env.json` Nav2 version capture are follow-ups. See the 2026-05-29 Nav2 ADR. |
| 2026-05-29 | teleop operator override (unit, `test_teleop.py`) | pending | fourth v0.2 slice: `Runtime.submit_teleop_command` runs a TELEOP command through `CommandGate`, emits COMMAND_ACCEPTED/REJECTED, and holds off autonomy for `teleop_hold_sec`; verified teleop overrides autonomy for the hold window then autonomy resumes; 138 unit tests pass (+5) | ROS-free operator path sharing `CommandGate`+`apply_external_command` with the bridge. sim-time hold → replayable. Unifying the bridge onto this API is a follow-up. See the 2026-05-29 teleop ADR. |
| 2026-05-29 | hardware diagnostics (unit, `test_diagnostics.py`) | pending | fifth v0.2 slice: `collect_diagnostics` folds estop/fall/failed/watchdog into per-agent OK\|WARN\|ERROR; `Runtime.diagnostics()` + `AgentToolApi.get_diagnostics()` read-models; periodic `DIAGNOSTICS` event when `diagnostics_interval_sec>0` (verified emitted; off by default); 147 unit tests pass (+9) | Gives the real-robot watchdog a consumer. Pure duck-typed collector — new health axes (battery, temp, comms) raise a level in one function. See the 2026-05-29 diagnostics ADR. |
| 2026-05-29 | v0.2.0a0 alpha tag | pending | session review + 動作確認 all green: 148 unit tests pass, all 5 slices verified e2e (replay OK, graceful nav2-no-server, teleop override, diagnostics levels), CI (`ci`/`docs`/`ros2`) green; pyproject + `__init__` bumped 0.1.0 → 0.2.0a0 | Marks the v0.2 groundwork milestone. Alpha: real-robot loop, Nav2 controller delegation, watchdog auto-poll remain follow-ups. Fixed one review finding (nav2 CLI unavailability now exits 4, not a traceback). |

## Failed Experiments

No failed experiments recorded yet.

## Known Flaky Scenarios

No flaky scenarios recorded yet.

## Reproduction Commands

```bash
gnav run examples/scenarios/smoke.yaml --fast --record
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/unit

# ROS 2 bridge end-to-end
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2_ws/src --packages-select genesis_nav_msgs genesis_nav_ros genesis_nav_bringup
source ros2_ws/install/setup.bash
gnav run examples/scenarios/smoke.yaml --fast --ros
```

## Benchmark History

Benchmark history starts once v0.1 scenario execution is wired to Genesis.
