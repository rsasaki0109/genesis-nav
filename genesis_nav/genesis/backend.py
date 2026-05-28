"""Genesis backend that owns the scene and per-tick physics step.

The backend keeps Genesis-specific code in one place. Runtime hooks call
`backend.step(dt_sec)` after each `runtime.step()` so the physics engine and
the runtime clock stay in lockstep.

`build_genesis_backend` is the only entry point that imports Genesis. If
Genesis is not installed, the function raises `GenesisNotAvailableError` with
an actionable hint instead of failing deep inside the runtime.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from genesis_nav.benchmarks.scenario import Scenario
from genesis_nav.core.agent import AgentSpec
from genesis_nav.core.embodiment import EmbodimentAdapter
from genesis_nav.genesis.adapter import GenesisDiffDriveAdapter
from genesis_nav.genesis.world_loader import WorldEntry, load_world_entry


class GenesisNotAvailableError(RuntimeError):
    """Raised when --backend genesis is requested but Genesis is missing."""


@dataclass
class GenesisBackend:
    """Owns a scene plus the per-agent adapters that populate it."""

    scene: Any
    world: WorldEntry
    adapters: dict[str, GenesisDiffDriveAdapter] = field(default_factory=dict)

    def step(self, dt_sec: float) -> None:
        del dt_sec
        scene = self.scene
        step_fn = getattr(scene, "step", None)
        if callable(step_fn):
            step_fn()

    def reset(self) -> None:
        scene = self.scene
        reset_fn = getattr(scene, "reset", None)
        if callable(reset_fn):
            reset_fn()

    def spawn(self, spec: AgentSpec) -> EmbodimentAdapter:
        entity = self.world.spawn_diff_drive(self.scene, spec)
        adapter = GenesisDiffDriveAdapter(agent_id=spec.agent_id, entity=entity)
        self.adapters[spec.agent_id] = adapter
        return adapter


def build_genesis_backend(scenario: Scenario) -> GenesisBackend:
    """Build a Genesis backend for the given scenario.

    Raises `GenesisNotAvailableError` if Genesis cannot be imported. The world
    entry point is responsible for its own Genesis imports so callers can run
    the world's `build_scene` only when this function decides Genesis is
    actually usable.
    """

    _require_genesis()
    world = load_world_entry(scenario.world)
    scene = world.build_scene(scenario.seed)
    return GenesisBackend(scene=scene, world=world)


def _require_genesis() -> None:
    try:
        import genesis  # noqa: F401
    except ImportError as exc:
        raise GenesisNotAvailableError(
            "Genesis is not installed. Install it from https://genesis-world.readthedocs.io"
            " (pip install genesis-world) before using --backend genesis."
        ) from exc


__all__ = ["GenesisBackend", "GenesisNotAvailableError", "build_genesis_backend"]
