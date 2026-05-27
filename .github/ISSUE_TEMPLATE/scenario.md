---
name: Scenario proposal
about: Add a new scenario or extend an existing one
title: "[scenario] "
labels: "scenario"
assignees: ""
---

## Scenario

Working name and one-line summary.

## Where it lives

- [ ] `examples/scenarios/<name>.yaml`
- [ ] `benchmarks/<suite>/<name>.yaml`

## Agents

Number, types, capabilities.

## Tasks

What are the agents asked to do?

## World

Existing world script under `examples/worlds/`, or new world that needs to be
added? If new, describe the geometry briefly.

## Metrics

Which metrics matter for this scenario? (See `docs/benchmarks.md` for the
shared vocabulary.)

## Reproducibility

- [ ] Deterministic seed
- [ ] No wall-clock or external dependencies
- [ ] Will produce identical `metrics.json` on the declared keys across runs

## Docs to update

- [ ] `docs/experiments.md` (append row after first green run)
- [ ] `docs/contributing_scenarios.md` only if you change the contribution
      contract
- [ ] `docs/interfaces.md` only if a new event/metric/schema field is
      introduced
