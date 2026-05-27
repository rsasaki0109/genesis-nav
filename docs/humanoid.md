# Humanoid Shell

The v0.1 humanoid agent type is a *navigation-intent shell*. It exists so
that scenarios, frame conventions, fleet plumbing, ROS interfaces, and the
fall-detected safety stop can be exercised end-to-end without taking on
whole-body control.

## What the shell does

- Adds the `humanoid` embodiment type to scenarios.
- Treats velocity commands as base-frame navigation intent and integrates
  the pelvis-projected base pose with planar kinematics
  (`genesis_nav.humanoid.HumanoidIntentAdapter`).
- Carries `pelvis`, `left_foot`, and `right_foot` frame names on
  `FrameSpec` for humanoid agents only.
- Surfaces `fall_detected` (plus `fall_reason` and `balance_margin`) from
  the adapter to the runtime.
- On the rising edge of `fall_detected`, the runtime emits `FALL_DETECTED`
  followed by `SAFETY_STOP` with `data.reason="fall_detected"` and
  `data.source="humanoid_adapter"`, then sets `emergency_stopped=True`.
  The existing emergency-stop branch handles `adapter.stop` and
  `COMMAND_REJECTED`.

## Non-goals (v0.1)

- No whole-body locomotion or torque control.
- No gait or footstep planning.
- No balance estimator. `fall_detected` is set by the adapter (in tests via
  `HumanoidIntentAdapter.trigger_fall`, in real backends via the balance
  estimator the backend provides).
- No biomechanical or contact simulation.

These are intentionally deferred. The shell is the right place to confirm
that the runtime, command gate, observability, and ROS contracts cope with
humanoid frame conventions and the fall-detected safety pathway. A real
locomotion adapter is a future workstream.

## Example scenario

`examples/scenarios/humanoid_nav_intent.yaml`:

```yaml
scenario_id: humanoid_nav_intent
seed: 42
world: examples/worlds/warehouse_small.py
agents:
  - id: humanoid_001
    type: humanoid
    spawn: [0.0, 0.0, 0.0]
    frames:
      map: map
      odom: humanoid_001/odom
      base: humanoid_001/base_link
      pelvis: humanoid_001/pelvis
      left_foot: humanoid_001/left_foot
      right_foot: humanoid_001/right_foot
    capabilities: [navigate_intent, stop, report_pose]
tasks:
  - id: nav_intent_001
    type: navigate_to_pose
    agent: humanoid_001
    goal: [2.0, 1.0, 0.0]
```

Run with:

```bash
gnav run examples/scenarios/humanoid_nav_intent.yaml --fast
```

## Adapter contract for backends

A backend humanoid adapter must implement `EmbodimentAdapter` and expose:

- `fall_detected: bool` (required)
- `fall_reason: str` (optional, copied into the event payload)
- `balance_margin: float` (optional, copied into the event payload)

The runtime treats all three with `getattr(..., default)`, so adapters that
do not expose them simply never fire a fall event. No subclass relationship
is required.
