---
name: Robot adapter
about: Propose a new robot model or extend an existing adapter
title: "[robot] "
labels: "robot-adapter"
assignees: ""
---

## Robot

Make, model, and form factor (diff drive, ackermann, omni, humanoid, etc.).

## Why it matters

What scenario, benchmark, or downstream consumer needs this robot? If this is
speculative, mark it as such and we will keep it on the backlog.

## Kinematics / dynamics

- Wheelbase / footprint:
- Max linear velocity:
- Max angular velocity:
- Sensor frames (planned):

## ROS 2 surface

- Namespace pattern: `/robot_<n>`
- Required topics already present in `docs/interfaces.md`?
  - [ ] Yes — list any new ones below
  - [ ] No — list everything below

```
<new topic / message / service / action list>
```

## Authority

- Default `authority.mode`:
- Default `command_ttl_ms`:
- Any non-default `CommandGate` allowance? If so, justify.

## Files likely to change

- `genesis_nav/robots/<adapter>.py`
- `genesis_nav/scenario/agents.py` (registration)
- Tests under `tests/unit/`
- `docs/interfaces.md`

## Reference

Manufacturer docs, prior art in other simulators, real hardware datasheet.
