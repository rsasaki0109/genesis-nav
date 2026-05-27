---
name: Benchmark proposal
about: Propose a new benchmark scenario, suite, or expectation predicate
title: "[bench] "
labels: "benchmark"
assignees: ""
---

## Suite

Which benchmark suite does this belong to? (`benchmarks/nav_basic`,
`benchmarks/multi_agent`, `benchmarks/runtime`, `benchmarks/humanoid`, or a
new suite.)

## Scenario summary

What is the scenario? Topology, agent count, task shape.

## Why it is worth running

What capability does it exercise that current benchmarks don't catch?

## Expectation predicates

List the predicates you intend to assert against (see
`docs/benchmarks.md` for the vocabulary). Example:

- `success_rate >= 0.95`
- `time_to_goal_mean <= 12.0`
- `collision_count == 0`

## Reproducibility

- [ ] Deterministic seed declared
- [ ] No wall-clock dependencies
- [ ] No external network calls

## Docs to update

- [ ] `docs/benchmarks.md`
- [ ] `docs/experiments.md` (append a row after first green run)
