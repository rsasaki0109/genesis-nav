"""Warehouse smoke world for the Genesis backend.

This module follows the genesis-nav world contract:

```python
def build_scene(seed: int) -> Scene: ...
def spawn_diff_drive(scene, spec) -> Any: ...
```

It targets the real Genesis 1.0 API (verified on genesis-world 1.0.0):
`gs.init(backend=...)` once per process, `gs.Scene(show_viewer=False)`, entities
added via `scene.add_entity(gs.morphs.*)`, and `scene.build()` after every
entity is added (the backend calls `build()` once all agents are spawned).

Set ``GENESIS_NAV_SPAWN_URDF=1`` to spawn the bundled diff-drive URDF with
wheel-joint velocity control instead of a kinematic box.

Genesis is imported lazily so the file can be loaded by the scenario tooling
even when Genesis is not installed (the load only fails when `--backend
genesis` is actually requested).
"""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any

WORLD_ID = "warehouse_small"
WORLD_SIZE_M = (10.0, 6.0)

_DIFF_DRIVE_URDF = (
    Path(__file__).resolve().parents[1] / "robots" / "diff_drive.urdf"
)

# Genesis must be initialized exactly once per process, before any Scene is
# constructed. Guarded here so repeated build_scene calls are safe.
_GENESIS_INITIALIZED = False


def build_scene(seed: int) -> Any:
    """Build a Genesis scene with a flat ground plane and warehouse fixtures."""

    gs = _require_genesis()
    _ensure_init(gs, seed)
    scene = gs.Scene(show_viewer=False)
    scene.add_entity(gs.morphs.Plane())
    _add_perimeter_walls(scene, gs)
    _add_shelves(scene, gs)
    return scene


def spawn_diff_drive(scene: Any, spec: Any) -> Any:
    """Add a differential-drive body for ``spec`` and return the Genesis entity."""

    gs = _require_genesis()
    spawn = spec.spawn or (0.0, 0.0, 0.0)
    yaw = float(spawn[2])
    pos = (float(spawn[0]), float(spawn[1]), 0.1)
    if _spawn_urdf_enabled():
        entity = _spawn_urdf(scene, gs, pos=pos, yaw=yaw)
        if entity is not None:
            return entity
    box = gs.morphs.Box(
        size=(0.4, 0.4, 0.2),
        pos=pos,
    )
    return scene.add_entity(box)


def _spawn_urdf_enabled() -> bool:
    return os.environ.get("GENESIS_NAV_SPAWN_URDF", "0") == "1"


def _spawn_urdf(scene: Any, gs: Any, *, pos: tuple[float, float, float], yaw: float) -> Any | None:
    if not _DIFF_DRIVE_URDF.is_file():
        return None
    try:
        half = 0.5 * yaw
        quat = (math.cos(half), 0.0, 0.0, math.sin(half))
        morph = gs.morphs.URDF(
            file=str(_DIFF_DRIVE_URDF),
            pos=pos,
            quat=quat,
        )
        return scene.add_entity(morph)
    except Exception:
        return None


def _add_perimeter_walls(scene: Any, gs: Any) -> None:
    width, height = WORLD_SIZE_M
    wall_h = 0.6
    wall_t = 0.12
    scene.add_entity(
        gs.morphs.Box(
            size=(width, wall_t, wall_h),
            pos=(width * 0.5, wall_t * 0.5, wall_h * 0.5),
        )
    )
    scene.add_entity(
        gs.morphs.Box(
            size=(width, wall_t, wall_h),
            pos=(width * 0.5, height - wall_t * 0.5, wall_h * 0.5),
        )
    )
    scene.add_entity(
        gs.morphs.Box(
            size=(wall_t, height, wall_h),
            pos=(wall_t * 0.5, height * 0.5, wall_h * 0.5),
        )
    )
    scene.add_entity(
        gs.morphs.Box(
            size=(wall_t, height, wall_h),
            pos=(width - wall_t * 0.5, height * 0.5, wall_h * 0.5),
        )
    )


def _add_shelves(scene: Any, gs: Any) -> None:
    """Static shelf blocks along the main aisle (visual + collision)."""

    shelf_h = 1.0
    shelf_w = 0.8
    shelf_d = 0.35
    placements = (
        (2.5, 1.2),
        (2.5, 4.8),
        (6.5, 1.2),
        (6.5, 4.8),
    )
    for x, y in placements:
        scene.add_entity(
            gs.morphs.Box(
                size=(shelf_w, shelf_d, shelf_h),
                pos=(x, y, shelf_h * 0.5),
            )
        )


def _ensure_init(gs: Any, seed: int) -> None:
    # Genesis allows exactly one gs.init() per process. The module-level guard
    # is not enough on its own because the world loader imports this file under
    # a fresh module name per scenario, so the global resets; we therefore also
    # tolerate Genesis's own "already initialized" signal. Either way init runs
    # at most once per process.
    global _GENESIS_INITIALIZED
    if _GENESIS_INITIALIZED or _genesis_is_initialized(gs):
        return
    # Prefer GPU when available; fall back to CPU. gs.gpu may be absent on
    # CPU-only builds.
    backend = getattr(gs, "gpu", None) or getattr(gs, "cpu", None)
    try:
        gs.init(seed=seed, backend=backend)
    except Exception as exc:  # pragma: no cover - depends on Genesis internals
        if "already initialized" not in str(exc).lower():
            raise
    _GENESIS_INITIALIZED = True


def _genesis_is_initialized(gs: Any) -> bool:
    """Best-effort check of Genesis's own initialized state."""
    for attr in ("_initialized", "is_initialized", "initialized"):
        val = getattr(gs, attr, None)
        if callable(val):
            try:
                return bool(val())
            except Exception:
                continue
        if isinstance(val, bool):
            return val
    return False


def _require_genesis():
    try:
        import genesis as gs  # type: ignore[import-not-found]
    except ImportError as exc:  # pragma: no cover - exercised when genesis is missing
        raise RuntimeError(
            "Genesis is not installed; cannot build scene for warehouse_small. "
            "Install via `pip install genesis-world` or use --backend fallback."
        ) from exc
    return gs
