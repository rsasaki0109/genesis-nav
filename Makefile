# genesis-nav developer convenience targets.
# These wrap commands already documented in CONTRIBUTING.md and
# docs/contributing_scenarios.md; the Makefile is just a shortcut.

.PHONY: help test smoke bench demo-gif

help:
	@echo "make test       — run the unit test suite"
	@echo "make smoke      — run the smoke scenario fast"
	@echo "make bench      — run the nav_basic benchmark suite"
	@echo "make demo-gif   — record the README demo GIF (needs asciinema + agg)"

test:
	PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/unit

smoke:
	gnav run examples/scenarios/smoke.yaml --fast --record

bench:
	gnav bench --run benchmarks/nav_basic

demo-gif:
	bash scripts/make_demo_gif.sh
