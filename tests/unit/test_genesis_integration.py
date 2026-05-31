"""Real-Genesis integration tests.

Skipped automatically when `genesis` is not importable, so the core unit suite
stays runnable without it (mirrors the rclpy-gated bridge tests). When Genesis
*is* present (e.g. the dedicated venv), these drive a real scene end to end:
`gs.init` -> `Scene` -> spawn -> `build()` -> step -> pose readback.
"""

from __future__ import annotations

from pathlib import Path

import pytest

pytest.importorskip("genesis")

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.runtime import Runtime
from genesis_nav.genesis.backend import build_genesis_backend
from genesis_nav.observability.events import JsonlEventWriter

SMOKE = Path("examples/scenarios/smoke.yaml")


def test_genesis_backend_builds_and_steps(tmp_path: Path) -> None:
    scenario = load_scenario(SMOKE)
    backend = build_genesis_backend(scenario)
    with JsonlEventWriter(tmp_path / "e.jsonl") as events:
        runtime = Runtime.from_scenario(
            scenario, events, adapter_factory=backend.spawn
        )
        backend.finalize()  # scene.build() after all spawns
        for task in scenario.tasks:
            runtime.submit_task(task, episode_id="ep")

        def on_step(_sim_time: float) -> None:
            backend.step(runtime.clock.step_sec)

        metrics = runtime.run_until_idle(
            episode_id="ep", max_sim_seconds=40.0, on_step=on_step
        )

    # The agent reached its goal under a real, built Genesis scene.
    assert metrics.summary()["success_rate"] == 1.0
    # The Genesis adapter read pose back from the real entity.
    pose = runtime.adapters["robot_001"].read_pose()
    assert pose[0] > 0.5  # advanced toward the (2, 1) goal


def test_genesis_backend_rejects_spawn_after_build(tmp_path: Path) -> None:
    scenario = load_scenario(SMOKE)
    backend = build_genesis_backend(scenario)
    spec = scenario.agents[0]
    backend.spawn(spec)
    backend.finalize()
    with pytest.raises(RuntimeError):
        backend.spawn(spec)
