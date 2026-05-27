---
name: Bug report
about: Something is broken or regressed
title: "[bug] "
labels: "bug"
assignees: ""
---

## What happened

## What you expected

## Reproduction

Minimum command(s) and / or scenario YAML to trigger the issue:

```bash
gnav run examples/scenarios/<...> --fast
```

If the bug shows up in tests:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/unit -k <pattern>
```

## Run artifacts

If you have a `runs/<timestamp>_<id>/` directory, attach (or paste the
relevant slice of):

- `report.md`
- `events.jsonl` tail
- `metrics.json`

## Environment

Output of `gnav doctor`:

```
<paste here>
```

OS, Python version, ROS 2 distro (if used), Genesis version (if used).

## Severity

- [ ] Blocks the v0.1 acceptance criteria in `PLAN.md`
- [ ] Wrong runtime behavior with workaround
- [ ] Cosmetic / docs
