"""Genesis adapter boundary.

Importing this package never imports Genesis itself. Genesis-specific imports
happen inside `genesis_nav.genesis.backend.build_genesis_backend` and inside
each world entry point so the core runtime stays Genesis-free.
"""

from genesis_nav.genesis.adapter import GenesisDiffDriveAdapter
from genesis_nav.genesis.backend import (
    GenesisBackend,
    GenesisNotAvailableError,
    build_genesis_backend,
)
from genesis_nav.genesis.world_loader import (
    GenesisWorldLoader,
    WorldEntry,
    load_world_entry,
)

__all__ = [
    "GenesisBackend",
    "GenesisDiffDriveAdapter",
    "GenesisNotAvailableError",
    "GenesisWorldLoader",
    "WorldEntry",
    "build_genesis_backend",
    "load_world_entry",
]
