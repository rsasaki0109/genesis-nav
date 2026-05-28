from pathlib import Path

import pytest

from genesis_nav.genesis.world_loader import (
    GenesisWorldLoader,
    WorldEntry,
    load_world_entry,
)


def test_loader_reads_world_from_file(tmp_path: Path) -> None:
    world_file = tmp_path / "tiny.py"
    world_file.write_text(
        "WORLD_ID = 'tiny'\n"
        "def build_scene(seed):\n"
        "    return {'seed': seed}\n"
        "def spawn_diff_drive(scene, spec):\n"
        "    return {'agent_id': spec.agent_id}\n",
        encoding="utf-8",
    )

    entry = load_world_entry(str(world_file))
    assert isinstance(entry, WorldEntry)
    assert entry.build_scene(7) == {"seed": 7}


def test_loader_requires_build_scene(tmp_path: Path) -> None:
    world_file = tmp_path / "noscene.py"
    world_file.write_text("WORLD_ID = 'x'\n", encoding="utf-8")
    entry = load_world_entry(str(world_file))
    with pytest.raises(AttributeError):
        _ = entry.build_scene


def test_loader_requires_spawn(tmp_path: Path) -> None:
    world_file = tmp_path / "no_spawn.py"
    world_file.write_text(
        "def build_scene(seed):\n    return {}\n",
        encoding="utf-8",
    )
    entry = load_world_entry(str(world_file))
    with pytest.raises(AttributeError):
        _ = entry.spawn_diff_drive


def test_loader_rejects_missing_file() -> None:
    with pytest.raises(FileNotFoundError):
        load_world_entry("examples/worlds/does_not_exist.py")


def test_loader_describe_round_trip() -> None:
    loader = GenesisWorldLoader(world="examples/worlds/warehouse_small.py", seed=3)
    assert loader.describe() == {
        "world": "examples/worlds/warehouse_small.py",
        "seed": 3,
    }
