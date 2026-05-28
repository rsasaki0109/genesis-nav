---
name: Architecture proposal
about: Propose an architectural change, boundary shift, or new ADR
title: "[arch] "
labels: "architecture"
assignees: ""
---

## Context

What is the current behavior and where does it live?

## Proposal

What should change? Be specific about module boundaries
(`genesis_nav/...`, `ros2_ws/src/...`).

## Why this matters

What problem does the current shape cause? Concrete examples, ideally with a
link to the scenario, benchmark, or PR that exposed it.

## Tradeoffs

What does this proposal cost? What does it foreclose?

## Alternatives considered

- Alt 1 — why rejected
- Alt 2 — why rejected

## Affected interfaces

- [ ] Public Python API (`genesis_nav/__init__.py`, public modules)
- [ ] ROS 2 topics / messages / actions
- [ ] Scenario schema
- [ ] Task schema
- [ ] Runtime event schema
- [ ] None of the above

## ADR plan

If accepted, this should land as an ADR in `docs/decisions.md`. Draft headline:

> YYYY-MM-DD: <one-sentence decision>
