# Launch Post Drafts

Drafts for the v0.1 launch on ROS Discourse and the Genesis community
channels. Both follow the same shape: what it is, what works today, why it
exists, where to push back.

Tone: factual, robotics-first, no marketing adjectives. The audience can
smell hype from across the room.

---

## ROS Discourse — `general / next-gen-ros / showcase`

**Subject:** `genesis-nav: a ROS 2-native runtime for Genesis World (v0.1)`

> We're sharing v0.1 of `genesis-nav`, a ROS 2-native embodied runtime
> infrastructure for Genesis World. It's not a Nav2 replacement and not an
> RL framework — it's the runtime layer between policy code and a real
> robot deployment.
>
> **What works today**
>
> - Scenario YAML drives the runtime; one command runs it end-to-end:
>   `gnav run examples/scenarios/smoke.yaml --fast --record`.
> - Each run writes a directory with `events.jsonl`, `metrics.json`,
>   `env.json`, and `report.md`. The same `events.jsonl` is the input to
>   `gnav replay`.
> - With `--ros`, the runtime mirrors to `/clock`, per-agent `/odom` and
>   `/state`, tf, and a `/genesis_nav/events` topic with appropriate QoS.
> - Multi-agent works: see `examples/scenarios/warehouse_10_agents.yaml`
>   (10 agents, dispatcher-routed tasks).
> - `gnav bench --run benchmarks/<suite>` runs scenarios with predicate
>   assertions (see `docs/benchmarks.md`).
> - Safety contract: AI agents go through `AgentToolApi`; all actuator
>   commands pass through `CommandGate` with authority + requester +
>   timestamp metadata.
>
> **Why this exists**
>
> The gap we kept hitting: simulator demos stop at "a policy moved a
> robot." Real robotics needs reproducible scenarios, stable interfaces,
> recorded evidence, fleet coordination, and a deployment story. Nav2 has
> the navigation stack; Isaac Lab has the RL stack; Genesis has the
> simulator. We wanted the runtime layer that connects them and is
> honest about safety boundaries.
>
> **What we want from you**
>
> - Eyes on `docs/interfaces.md` — that's the public contract.
> - Push back on the Nav2 boundary in `docs/decisions.md`.
> - Tell us where the ROS 2 surface is wrong.
> - File scenarios that exercise things we don't.
>
> Repo: <https://github.com/rsasaki0109/genesis-nav>
> Quickstart: README
> Contributing: `CONTRIBUTING.md` + `docs/good_first_issues.md`
>
> Happy to answer questions in-thread.

---

## Genesis Community — Discord `#showcase` and GitHub Discussions

**Subject:** `genesis-nav v0.1 — runtime, replay, and ROS 2 bridge for Genesis`

> Hey Genesis folks — sharing v0.1 of `genesis-nav`. Think of it as the
> "what do I do with my Genesis scene once a policy is running?" layer.
>
> **The pitch in one line**
>
> Genesis is the simulator, ROS 2 is the deployment contract,
> `genesis-nav` is the runtime that makes those two compose without
> custom glue per project.
>
> **What you can run today**
>
> - `gnav doctor` checks Genesis + ROS 2 availability; the smoke scenario
>   runs without either (in-memory kinematics fallback).
> - `gnav run examples/scenarios/smoke.yaml --fast --record` —
>   single-agent navigate-to-pose with a grid-aware planner.
> - `--ros` flag publishes per-agent state, odom, tf, and an events
>   topic with sensible QoS.
> - Run artifacts are deterministic; we already use them as reviewable
>   evidence in PRs (`docs/experiments.md`).
>
> **What's intentionally not there**
>
> - No Nav2 plugin layer in v0.1 — direct, understandable paths first.
> - No humanoid whole-body controller — `docs/humanoid.md` explains the
>   shell-only scope.
> - No RL training loop. We integrate with policies; we don't train them.
>
> **How to help**
>
> - Try the smoke scenario, file what surprised you.
> - The good-first-issues list (`docs/good_first_issues.md`) has 10
>   small things from "add a holonomic adapter" to "document the v0.2
>   boundary".
> - Robot adapter contributions especially welcome — drop an issue with
>   the `robot_adapter` template.
>
> Repo: <https://github.com/rsasaki0109/genesis-nav>
> Demo: README GIF, longer walkthrough at <youtube TBD>.
> Roadmap: `docs/roadmap.md` and `PLAN.md`.

---

## Pre-flight checklist (before posting)

- [ ] README GIF in place.
- [ ] `gnav doctor` runs cleanly on a fresh clone with `pip install -e ".[dev]"`.
- [ ] Smoke + warehouse scenarios green on the latest commit.
- [ ] `docs/experiments.md` has a row dated the day of posting.
- [ ] GitHub Discussions enabled with categories: `Q&A`,
      `Show & Tell`, `Architecture`, `Robot adapters`.
- [ ] Issue templates render in the GitHub UI (visit
      `/issues/new/choose`).
- [ ] First wave of good-first-issues actually filed (not just listed in
      docs).

## Response plan

- For "is this Nav2-compatible?" — point at `docs/decisions.md` ADR on
  the Nav2 boundary; offer to discuss in an architecture issue.
- For "can I run this on real hardware?" — v0.1 is sim-first; the ROS 2
  contract is the path. Real-robot adapters are planned for v0.2.
- For "why not just use X?" — answer with the comparison table in the
  README and a one-line "we may be wrong; file an architecture issue".
