"""Structured runtime event log."""

from __future__ import annotations

import json
from collections import deque
from collections.abc import Iterable
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Protocol, TextIO

BENCHMARK_REPORT = "BENCHMARK_REPORT"

POST_SCENARIO_EVENTS: frozenset[str] = frozenset({BENCHMARK_REPORT})


@dataclass(frozen=True)
class RuntimeEvent:
    ts: float
    episode_id: str
    event: str
    agent_id: str = ""
    task_id: str = ""
    data: dict[str, Any] = field(default_factory=dict)


class EventSink(Protocol):
    def write(
        self,
        *,
        ts: float,
        episode_id: str,
        event: str,
        agent_id: str = "",
        task_id: str = "",
        data: dict[str, Any] | None = None,
    ) -> None: ...


class FanoutEventSink:
    """Forwards each event to every registered sink in order."""

    def __init__(self, sinks: Iterable[EventSink]) -> None:
        self._sinks: list[EventSink] = list(sinks)

    def add(self, sink: EventSink) -> None:
        self._sinks.append(sink)

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
        payload = data or {}
        for sink in self._sinks:
            sink.write(
                ts=ts,
                episode_id=episode_id,
                event=event,
                agent_id=agent_id,
                task_id=task_id,
                data=payload,
            )


class RingBufferEventSink:
    """Capacity-bounded in-memory tail of runtime events.

    Used by the AI tool API to serve `get_recent_events` without re-reading
    `events.jsonl`. Older records are dropped once the buffer is full.
    """

    def __init__(self, capacity: int = 1024) -> None:
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        self._buffer: deque[RuntimeEvent] = deque(maxlen=capacity)

    @property
    def capacity(self) -> int:
        max_len = self._buffer.maxlen
        assert max_len is not None
        return max_len

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
        self._buffer.append(
            RuntimeEvent(
                ts=ts,
                episode_id=episode_id,
                event=event,
                agent_id=agent_id,
                task_id=task_id,
                data=dict(data or {}),
            )
        )

    def snapshot(self) -> list[RuntimeEvent]:
        return list(self._buffer)

    def filter(
        self,
        *,
        event: str | None = None,
        agent_id: str | None = None,
        task_id: str | None = None,
        since_ts: float | None = None,
        limit: int = 100,
    ) -> list[RuntimeEvent]:
        if limit <= 0:
            return []
        matches: list[RuntimeEvent] = []
        for record in self._buffer:
            if event is not None and record.event != event:
                continue
            if agent_id is not None and record.agent_id != agent_id:
                continue
            if task_id is not None and record.task_id != task_id:
                continue
            if since_ts is not None and record.ts < since_ts:
                continue
            matches.append(record)
        if len(matches) > limit:
            return matches[-limit:]
        return matches


class JsonlEventWriter:
    def __init__(self, path: Path) -> None:
        self.path = path
        self._handle: TextIO | None = None

    def __enter__(self) -> "JsonlEventWriter":
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self.path.open("w", encoding="utf-8")
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        if self._handle:
            self._handle.close()
            self._handle = None

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
        if self._handle is None:
            raise RuntimeError("JsonlEventWriter must be used as a context manager")
        record = RuntimeEvent(
            ts=ts,
            episode_id=episode_id,
            agent_id=agent_id,
            event=event,
            task_id=task_id,
            data=data or {},
        )
        self._handle.write(json.dumps(asdict(record), sort_keys=True) + "\n")
        self._handle.flush()


def append_runtime_event(path: Path, event: RuntimeEvent) -> None:
    """Append one serialized event record to an existing JSONL file."""

    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(asdict(event), sort_keys=True) + "\n")
