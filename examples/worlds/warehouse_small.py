"""Warehouse smoke world for the v0.1 Genesis adapter.

This module follows the genesis-nav world contract:

```python
def build_scene(seed: int) -> Scene: ...
def spawn_diff_drive(scene, spec) -> Any: ...
```

Genesis is imported lazily so the file can be loaded by the scenario tooling
even when Genesis is not installed (the load only fails when `--backend
genesis` is actually requested).
"""

from __future__ import annotations

from typing import Any


WORLD_ID = "warehouse_small"
WORLD_SIZE_M = (10.0, 6.0)


def build_scene(seed: int) -> Any:
    """Build a Genesis scene seeded for reproducibility.

    The current implementation is intentionally minimal: a flat ground plane
    sized for the warehouse footprint. v0.2 will add aisle walls, shelves, and
    docking pads.
    """

    gs = _require_genesis()
    scene = gs.Scene(seed=seed) if "seed" in _signature(gs.Scene) else gs.Scene()
    plane = getattr(gs.morphs, "Plane", None)
    if plane is not None:
        scene.add_entity(plane())
    return scene


def spawn_diff_drive(scene: Any, spec: Any) -> Any:
    """Spawn a differential-drive body for `spec` and return the Genesis entity."""

    gs = _require_genesis()
    spawn = spec.spawn or (0.0, 0.0, 0.0)
    box = gs.morphs.Box(size=(0.4, 0.4, 0.2), pos=(spawn[0], spawn[1], 0.1))
    entity = scene.add_entity(box, name=spec.agent_id)
    rotator = getattr(entity, "set_yaw", None)
    if callable(rotator):
        rotator(spawn[2])
    return entity


def _require_genesis():
    try:
        import genesis as gs  # type: ignore[import-not-found]
    except ImportError as exc:  # pragma: no cover - exercised when genesis is missing
        raise RuntimeError(
            "Genesis is not installed; cannot build scene for warehouse_small. "
            "Install via `pip install genesis-world` or use --backend fallback."
        ) from exc
    return gs


def _signature(callable_obj: Any) -> set[str]:
    try:
        import inspect

        return set(inspect.signature(callable_obj).parameters)
    except (TypeError, ValueError):
        return set()
