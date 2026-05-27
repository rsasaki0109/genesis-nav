# Genesis Integration

Genesis is the primary simulator. `genesis-nav` keeps Genesis-specific code in
`genesis_nav/genesis/` and exposes only the minimal runtime adapter surface.

Adapter responsibilities:

- world loading
- entity spawning
- pose and twist extraction
- sensor extraction
- command application
- collision query
- simulation step
- reset
- seed control

Responsibilities kept outside the Genesis adapter:

- fleet scheduling
- task planning
- AI-agent decisions
- ROS 2 QoS policy
- benchmark scoring
- human-readable experiment logs

## v0.1 Adapter Surface

`genesis_nav.genesis` exposes three boundary objects:

- `GenesisWorldLoader` / `load_world_entry` — resolves a scenario `world` field
  (either a `path/to/world.py` or a dotted module name) into a `WorldEntry`
  with `build_scene(seed)` and `spawn_diff_drive(scene, spec)` functions.
- `GenesisBackend` — owns the scene, calls `scene.step()` once per runtime
  tick via the `on_step` hook, and creates `GenesisDiffDriveAdapter` instances
  on demand.
- `GenesisDiffDriveAdapter` — implements `EmbodimentAdapter`; talks to the
  entity through `set_velocity` / `get_pose` (preferred) or attribute writes
  (fallback).

`build_genesis_backend(scenario)` is the only function that imports
`genesis`. If Genesis is not installed it raises `GenesisNotAvailableError`
with the `pip install genesis-world` hint also shown by `gnav doctor`.

## World File Contract

A world module must define:

```python
def build_scene(seed: int) -> Scene: ...
def spawn_diff_drive(scene, spec) -> Any: ...
```

`scene` and the returned entity stay opaque to the rest of genesis-nav. The
adapter expects the entity to expose either `set_velocity(linear_x, linear_y,
angular_z)` and `get_pose() -> (x, y, yaw)`, or attribute access
(`linear_velocity`, `angular_velocity`, `x`, `y`, `yaw`).

`examples/worlds/warehouse_small.py` is the v0.1 reference implementation. It
imports `genesis` lazily so scenarios can still load with the fallback
backend.

## Running with Genesis

```bash
gnav doctor                          # verify Genesis is detected
gnav run examples/scenarios/smoke.yaml --fast --backend genesis
```

Use `--backend fallback` (default) to run the deterministic in-memory
diff-drive integrator when Genesis is not installed.
