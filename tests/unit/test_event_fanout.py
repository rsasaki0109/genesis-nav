from dataclasses import dataclass, field
from typing import Any

from genesis_nav.observability.events import FanoutEventSink


@dataclass
class _Recorder:
    rows: list[dict[str, Any]] = field(default_factory=list)

    def write(
        self,
        *,
        ts: float,
        episode_id: str,
        event: str,
        agent_id: str = "",
        task_id: str = "",
        data: dict[str, Any] | None = None,
    ) -> None:
        self.rows.append(
            {
                "ts": ts,
                "episode_id": episode_id,
                "event": event,
                "agent_id": agent_id,
                "task_id": task_id,
                "data": dict(data or {}),
            }
        )


def test_fanout_event_sink_forwards_to_all_sinks() -> None:
    a, b = _Recorder(), _Recorder()
    sink = FanoutEventSink([a, b])
    sink.write(ts=1.0, episode_id="ep", event="TEST", agent_id="r1", data={"x": 1})
    sink.add(_Recorder())  # late-added sinks do not receive past events
    sink.write(ts=2.0, episode_id="ep", event="TEST2")

    assert len(a.rows) == 2
    assert len(b.rows) == 2
    assert a.rows[0]["event"] == "TEST"
    assert a.rows[0]["agent_id"] == "r1"
    assert a.rows[1]["event"] == "TEST2"
