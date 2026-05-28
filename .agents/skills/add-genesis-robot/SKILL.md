# Add Genesis Robot

Use this skill when adding a new robot embodiment or Genesis-side robot adapter.

## Workflow

1. Keep Genesis-specific code under `genesis_nav/genesis/`.
2. Add robot config under `configs/robots/`.
3. Add a minimal scenario under `examples/scenarios/`.
4. Ensure the robot has an `agent_id`, namespace, frames, capabilities, and
   `CommandGate` path.
5. Run:

```bash
gnav run examples/scenarios/<scenario>.yaml --fast
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/unit
```
