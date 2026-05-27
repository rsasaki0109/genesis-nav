# AI Agents

AI agents are operators and task planners. They are not actuator controllers.

The only Python surface AI agents are allowed to call against the runtime is
`genesis_nav.agent.AgentToolApi`. Construct it via `Runtime.tool_api(...)`:

```python
api = runtime.tool_api(scenario_id="warehouse", event_buffer=ring_buffer)
```

`event_buffer` is a `RingBufferEventSink` registered alongside the JSONL writer
in the runtime's event fanout. Without it, `get_recent_events` returns `[]`.

## Allowed Operations

| Method | Returns | Notes |
|---|---|---|
| `list_agents()` | `list[AgentSnapshot]` | Read-only snapshot of every agent |
| `get_world_state()` | `WorldSnapshot` | Scenario id, episode, sim time, pending tasks, lease count |
| `get_task_status(task_id)` | `TaskSnapshot \| None` | Status for queued, active, or terminal tasks |
| `get_recent_events(event=, agent_id=, task_id=, since_ts=, limit=)` | `list[RuntimeEvent]` | Filtered tail from the ring buffer |
| `submit_task(task, *, requester_id, trace_id=None)` | `TaskSnapshot` | Routes through `Runtime.submit_task` and the dispatcher |
| `pause_agent(agent_id, reason, *, requester_id)` | `None` | Sets `emergency_stopped=True`; emits `SAFETY_STOP` with `scope=agent` |
| `resume_agent(agent_id, *, requester_id)` | `None` | Clears `emergency_stopped`; emits `AGENT_RESUMED` |
| `stop_all(reason, *, requester_id)` | `None` | Stops every agent; emits `SAFETY_STOP` with `scope=all` |

## Safety Rules

- AI agents cannot publish `cmd_vel`. The tool API exposes no actuator method.
- AI-issued tasks must include `requester_id`. The API also stamps a
  `trace_id` if the caller does not provide one (UUID4 hex).
- `pause_agent`, `resume_agent`, and `stop_all` all require `requester_id`.
- AI agents cannot disable safety gates. `CommandGate` still rejects
  `AuthorityMode.AI` velocity commands by default.
- AI agents cannot delete logs or mutate active QoS/runtime config.
- Every AI-originated event carries `data.source = "ai_tool_api"` and the
  `requester_id` so the audit trail is unambiguous in `events.jsonl`.

## Example

```python
from genesis_nav.core.task import TaskSpec

task = TaskSpec(
    task_id="ai_pick_001",
    task_type="navigate_to_pose",
    goal=(4.0, 2.0, 0.0),
    constraints={"agent_selector": {"capabilities": ["navigate_2d"]}},
)
snapshot = api.submit_task(task, requester_id="planner_agent_v1")
# Later:
status = api.get_task_status(snapshot.task_id)
recent = api.get_recent_events(task_id=snapshot.task_id, limit=20)
```
