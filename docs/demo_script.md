# 2-Minute Demo Script

This document is the recording script + shot list for the first
`genesis-nav` demo video and the GIF that embeds in the README. Goal:
prove the v0.1 acceptance shape in 120 seconds — runtime, ROS 2 topics,
replay, and metrics.

## Audience

- Roboticists familiar with Nav2 or Gazebo who want to see what is
  different.
- Embodied AI researchers who already have a policy and want a runtime.
- Reviewers who clicked the README and have 30 seconds before they leave.

## What we have to show

1. One command to run a scenario.
2. The runtime emits structured events and metrics.
3. The same run mirrors to ROS 2 topics live.
4. `gnav replay` rebuilds the run from `events.jsonl`.
5. `gnav bench` runs the same scenario as a pass/fail benchmark.

## Shot list (2:00 target)

### 00:00 – 00:10 Hook
Voiceover: "Embodied AI has policies. Robots need runtime."
On screen: README hero, then a terminal prompt.

### 00:10 – 00:30 One command
```bash
gnav run examples/scenarios/smoke.yaml --fast --record
```
Show:
- Console output streaming `BEHAVIOR_STATE_CHANGED` and
  `PLAN_RESOLVED` lines.
- Final `success_rate=1.0` summary.

### 00:30 – 00:55 The run directory
`ls runs/<latest>/` then open:
- `report.md` (success, time-to-goal, sim steps)
- `events.jsonl` first 20 lines
- `metrics.json` pretty-printed
- `env.json` showing python / platform / git / ros_distro

Voiceover: "Every run is a directory. Replay it, diff it, ship it as
evidence."

### 00:55 – 01:25 ROS 2 mirror
Split screen: re-run with `--ros`, then in second pane:
```bash
ros2 topic list
ros2 topic echo /genesis_nav/events --once
ros2 topic echo /robot_001/odom --once
```
Voiceover: "ROS 2 is the public contract. The bridge ships the same
events to topics, with QoS that real robots want."

### 01:25 – 01:45 Replay
```bash
gnav replay runs/<latest>/events.jsonl --print-events
```
Show the same event stream replayed in order. Mention strict mode.

### 01:45 – 02:00 Bench + close
```bash
gnav bench --run benchmarks/nav_basic
```
Show the JSON summary: 1/1 passed. End on README hero + repo URL.

## GIF version (12 seconds, README hero)

Same content, compressed:

- 0–3s: `gnav run ... --fast` runs to success.
- 3–6s: `cat runs/<latest>/report.md`.
- 6–9s: `gnav replay ... --print-events` tail.
- 9–12s: `gnav bench --run benchmarks/nav_basic` green.

Encode at 1280×720, 12fps, ~3MB. Place at `docs/media/smoke_demo.gif`
and link from the README.

## Recording instructions

1. Use `examples/scenarios/smoke.yaml` (no host dependencies).
2. Clean terminal: `PS1='$ '`, large font, dark background.
3. Recreate the run dir each take so the timestamp prefix is fresh.
4. Use `asciinema` for the long-form video, then export. Use `vhs` or
   `terminalizer` for the GIF — the deterministic playback avoids the
   "typing speed varies" feel.
5. Do not include `gnav doctor` output unless the host has a fresh
   Genesis or ROS 2 install — the goal is "what works", not "what you
   need to install".

## What we are explicitly **not** showing

- Genesis viewer (not required for v0.1 demos).
- A whole-body humanoid controller (out of scope — see
  `docs/humanoid.md`).
- A policy training loop. We are the runtime, not the trainer.

## Distribution

- README hero GIF.
- Upload long-form to YouTube (unlisted first, then public after the
  launch posts go out — see `docs/launch_posts.md`).
- Twitter / X clip: trim to 60 seconds, lead with the ROS 2 mirror.
