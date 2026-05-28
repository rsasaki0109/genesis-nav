# Contributing to genesis-nav

Thanks for your interest. `genesis-nav` is a ROS 2-native embodied runtime for
Genesis World. We optimise for working runtime paths, reproducible scenarios,
and visible safety boundaries — not abstraction depth.

If you can spend five minutes before opening a PR, the project will reward you
with much faster review.

## Where to contribute

Most new contributors should start in one of these areas:

| Area | Where the code lives | Typical first PR |
|---|---|---|
| Scenarios | `examples/scenarios/*.yaml`, `genesis_nav/scenario/` | Add a YAML scenario with metrics + expectations |
| Benchmarks | `benchmarks/<suite>/`, `docs/benchmarks.md` | Add a scenario + expectation predicates to an existing suite |
| Robot adapters | `genesis_nav/robots/`, `genesis_nav/genesis/` | Add a kinematics model for a new robot |
| ROS 2 bridge | `genesis_nav/ros/`, `ros2_ws/src/` | Publish a new topic, mirror a runtime event, add a QoS profile |
| Observability | `genesis_nav/observability/`, `docs/replay.md` | Add a metric, an event type, or a replay predicate |
| Docs | `docs/`, `README.md` | Fix factual drift, add diagrams, document an existing module |

A curated list of suggested first issues lives in
[`docs/good_first_issues.md`](docs/good_first_issues.md).

## Project shape

Read these before changing public interfaces:

- `AGENTS.md` — module boundaries and the safety contract.
- `docs/interfaces.md` — every public ROS topic, action, runtime event, task
  schema, scenario schema. Update it in the same PR if any of these change.
- `docs/decisions.md` — short ADRs. Add one when you make an architectural
  tradeoff (e.g., introducing a new module, swapping a planner, picking a
  default QoS).
- `docs/experiments.md` — append a row when you add or change a scenario or
  benchmark.

If your PR touches a public interface but does not update `docs/interfaces.md`,
reviewers will ask you to add the update. The pull request template has the
checklist.

## Dev setup

```bash
git clone https://github.com/<your-fork>/genesis-nav.git
cd genesis-nav
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -e ".[dev]"
gnav doctor
```

`gnav doctor` reports backend availability (Genesis, ROS 2). The runtime works
without Genesis or ROS 2 — the smoke scenario uses an in-memory kinematics
fallback.

## Running tests

The minimum bar before opening a PR:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/unit
gnav run examples/scenarios/smoke.yaml --fast
```

If you touched ROS 2 packages:

```bash
source /opt/ros/jazzy/setup.bash
colcon test --base-paths ros2_ws/src \
  --packages-select genesis_nav_msgs genesis_nav_ros
```

If you touched benchmark code:

```bash
gnav bench --run benchmarks/nav_basic
```

## Safety boundary (read before sending actuator-adjacent PRs)

These rules are not style preferences. They guard the safety contract that
makes `genesis-nav` usable on real robots.

1. **AI agents do not publish actuator commands.** They go through
   `genesis_nav.agent.AgentToolApi`. See `docs/ai_agents.md` for the allowed
   surface.
2. **All actuator commands pass through `CommandGate`** with explicit
   `authority`, `requester`, and `timestamp` metadata.
3. **Replay and logging paths are first-class.** Do not gate them behind
   debug flags or remove them as a "cleanup".
4. **QoS, gates, and active runtime config are not mutable by AI tools.**
   Changes require an explicit human-authored config or PR.

If a change feels like it is widening the AI surface, write the ADR first and
get review on the boundary before writing the code.

## PR expectations

- **One topic per PR.** Reviewers will ask you to split a refactor that also
  ships a feature.
- **Include verification in the PR body.** Either paste the relevant test
  output or describe the manual reproduction.
- **Update docs in the same PR as the code change** (interfaces, experiments,
  decisions).
- **No AI-generated marketing language.** Keep PR descriptions factual and
  short.
- **Co-Authored-By trailers are optional.** The repository maintainers do not
  add them; you are not required to either.

## Commit style

Subject line: imperative mood, lowercase, under 70 chars
(`add stuck detector to runtime`). Body: explain *why*, not *what*; the diff
already shows what. Reference issues with `#<n>`.

## Issue templates

Use the right template — it speeds up triage:

- `task` — concrete implementation with acceptance criteria.
- `architecture` — proposes an ADR or boundary change.
- `benchmark` — proposes a benchmark scenario or expectation change.
- `robot_adapter` — proposes a new robot model or adapter.
- `scenario` — proposes a new scenario or extension to an existing one.
- `bug` — something is broken or regressed.

## Code of conduct

Be specific, be kind, attack ideas not people. We follow the Contributor
Covenant. Maintainers may close PRs or issues that violate the contract or
the safety boundary; we will explain why and point at the relevant doc.

## Questions

Open a GitHub Discussion or a `task` issue. For architecture questions,
prefer a draft `architecture` issue so the conversation can live alongside
the resulting ADR.
