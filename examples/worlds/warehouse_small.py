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

Genesis is imported lazily so the file can be loaded by the scenario tooling
even when Genesis is not installed (the load only fails when `--backend
genesis` is actually requested).
"""

from __future__ import annotations

from typing import Any

WORLD_ID = "warehouse_small"
WORLD_SIZE_M = (10.0, 6.0)

# Genesis must be initialized exactly once per process, before any Scene is
# constructed. Guarded here so repeated build_scene calls are safe.
_GENESIS_INITIALIZED = False


def build_scene(seed: int) -> Any:
    """Build a Genesis scene with a flat ground plane.

    Minimal warehouse footprint. Aisle walls and shelves are future work; the
    point of this slice is a *real* Genesis scene the runtime can step
    deterministically, not a furnished map.
    """

    gs = _require_genesis()
    _ensure_init(gs, seed)
    scene = gs.Scene(show_viewer=False)
    scene.add_entity(gs.morphs.Plane())
    return scene


def spawn_diff_drive(scene: Any, spec: Any) -> Any:
    """Add a differential-drive body for `spec` and return the Genesis entity.

    The body is a free rigid box; genesis-nav drives it kinematically through
    `GenesisDiffDriveAdapter` (set_pos / set_quat each tick), the honest model
    for a v0.x diff-drive base without an articulated drivetrain.
    """

    gs = _require_genesis()
    spawn = spec.spawn or (0.0, 0.0, 0.0)
    box = gs.morphs.Box(
        size=(0.4, 0.4, 0.2),
        pos=(float(spawn[0]), float(spawn[1]), 0.1),
    )
    return scene.add_entity(box)


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
