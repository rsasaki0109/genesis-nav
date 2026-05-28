"""Resolve a scenario world entry point into a callable scene builder.

A world file is a Python module that exposes:

```python
def build_scene(seed: int) -> Scene: ...
def spawn_diff_drive(scene, spec) -> Any: ...
```

`Scene` and the returned entity are intentionally opaque from the loader's
point of view -- the Genesis adapter knows how to drive them. Lazy importlib
loading means the loader works whether `world` is a `path/to/world.py` or a
dotted module like `genesis_nav_examples.worlds.warehouse_small`.
"""

from __future__ import annotations

import importlib
import importlib.util
import sys
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Any, Callable


WorldBuildFn = Callable[[int], Any]
SpawnFn = Callable[[Any, Any], Any]


@dataclass(frozen=True)
class WorldEntry:
    """A loaded world module ready to build scenes and spawn agents."""

    module: ModuleType
    source: str

    @property
    def build_scene(self) -> WorldBuildFn:
        fn = getattr(self.module, "build_scene", None)
        if fn is None:
            raise AttributeError(
                f"world '{self.source}' must define build_scene(seed) to be Genesis-ready"
            )
        return fn

    @property
    def spawn_diff_drive(self) -> SpawnFn:
        fn = getattr(self.module, "spawn_diff_drive", None)
        if fn is None:
            raise AttributeError(
                f"world '{self.source}' must define spawn_diff_drive(scene, spec)"
            )
        return fn


class GenesisWorldLoader:
    """Loads a world entry point either by file path or by dotted module name."""

    def __init__(self, world: str, seed: int) -> None:
        self.world = world
        self.seed = seed

    def describe(self) -> dict[str, object]:
        return {"world": self.world, "seed": self.seed}

    def load(self) -> WorldEntry:
        return load_world_entry(self.world)


def load_world_entry(world: str) -> WorldEntry:
    if not world:
        raise ValueError("scenario.world must be set to a path or dotted module name")
    path = Path(world)
    if path.suffix == ".py" or path.exists():
        return _load_from_path(path)
    return _load_from_dotted(world)


def _load_from_path(path: Path) -> WorldEntry:
    resolved = path.resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"world file not found: {path}")
    module_name = f"_genesis_nav_world_{resolved.stem}_{abs(hash(str(resolved)))}"
    spec = importlib.util.spec_from_file_location(module_name, resolved)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load world module from {resolved}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return WorldEntry(module=module, source=str(resolved))


def _load_from_dotted(name: str) -> WorldEntry:
    module = importlib.import_module(name)
    return WorldEntry(module=module, source=name)
