# Good First Issues

The original ten starter tasks are **closed as of v0.2.2** (2026-06-10).
Pick a follow-up below or browse [`docs/roadmap_v02.md`](roadmap_v02.md) for
v0.3+ direction.

Size key: **XS** half-day · **S** one to two days · **M** ~one week

---

## Closed in v0.2.x

| # | Task | Closed in |
|---|------|-----------|
| 1 | Holonomic robot adapter (`type: holonomic`) | 0.2.2 |
| 2 | `occupancy_grid.inflate_cells` | 0.2.2 |
| 3 | `BENCHMARK_REPORT` runtime event | 0.2.2 |
| 4 | `gnav doctor --json` | 0.2.2 |
| 5 | Rosbag record + `gnav replay --to-rosbag` | 0.2.2 |
| 6 | `charging_dock.yaml` + `dwell_sec` | 0.2.2 |
| 7 | Wilson `success_rate_ci` in metrics | 0.2.2 |
| 8 | Architecture Mermaid diagrams | 0.2.0 |
| 9 | `runtime.navigation.planner` selector | 0.2.0a0 |
| 10 | `docs/roadmap_v02.md` boundary doc | 0.2.0 |

---

## Open follow-ups (good next PRs)

### 1. Holonomic Genesis / real-robot backend

**Size:** S — **Template:** robot_adapter

The in-memory `HolonomicKinematics` adapter exists; wire it through
`--backend genesis` and `ros2_robot` (loopback transport already accepts
`linear_y`).

- Files: `genesis_nav/genesis/`, `genesis_nav/ros2_robot/`, tests.

### 2. Bench scenario for holonomic smoke

**Size:** XS — **Template:** scenario

Add `benchmarks/nav_basic/holonomic_smoke.yaml` guarding the new adapter.

- Files: scenario YAML, row in `docs/experiments.md`.

### 3. `success_rate_ci` bench predicates

**Size:** XS — **Template:** task

Add `success_rate_ci_low_min` (or similar) to `BenchmarkExpectation` so
multi-task suites can assert confidence bounds.

- Files: `genesis_nav/benchmarks/report.py`, tests, `docs/benchmarks.md`.

### 4. Smarter fleet priority

**Size:** M — **Template:** architecture

Replace lexicographic agent-id priority with goal-distance or RVO-style
deconfliction. See proximity ADRs and `docs/roadmap_v02.md`.

### 5. Non-loopback real-hardware transport

**Size:** M — **Template:** robot_adapter

Extend `Ros2RobotTransport` beyond loopback for a live `/odom` + `/cmd_vel`
edge on hardware.

---

## Picking the right one

- New to ROS 2? Start with **2** or **3**.
- Familiar with Python / sim? **1** or **2**.
- Strong on robotics / fleet? **4** or **5**.
