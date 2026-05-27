"""Agent registry and state models."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable

from genesis_nav.core.authority import AuthorityMode, parse_authority
from genesis_nav.core.lifecycle import LifecycleState
from genesis_nav.core.task import TaskStatus
from genesis_nav.navigation.behavior import BehaviorState


@dataclass(frozen=True)
class FrameSpec:
    map: str = "map"
    odom: str = "odom"
    base: str = "base_link"
    pelvis: str = ""
    left_foot: str = ""
    right_foot: str = ""

    @classmethod
    def from_mapping(cls, data: dict[str, Any] | None, agent_id: str) -> "FrameSpec":
        data = data or {}
        return cls(
            map=str(data.get("map", "map")),
            odom=str(data.get("odom", f"{agent_id}/odom")),
            base=str(data.get("base", f"{agent_id}/base_link")),
            pelvis=str(data.get("pelvis", "")),
            left_foot=str(data.get("left_foot", "")),
            right_foot=str(data.get("right_foot", "")),
        )


@dataclass(frozen=True)
class AgentSpec:
    agent_id: str
    embodiment: str
    namespace: str
    frames: FrameSpec = field(default_factory=FrameSpec)
    capabilities: tuple[str, ...] = ("navigate_2d", "stop", "report_pose")
    authority_mode: AuthorityMode = AuthorityMode.AUTONOMY
    command_ttl_ms: int = 200
    spawn: tuple[float, float, float] | None = None

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "AgentSpec":
        agent_id = str(data.get("id") or data.get("agent_id") or "").strip()
        if not agent_id:
            raise ValueError("agent entry requires 'id'")

        spawn = data.get("spawn")
        parsed_spawn: tuple[float, float, float] | None = None
        if spawn is not None:
            if not isinstance(spawn, list | tuple) or len(spawn) != 3:
                raise ValueError(f"agent '{agent_id}' spawn must be [x, y, yaw]")
            parsed_spawn = (float(spawn[0]), float(spawn[1]), float(spawn[2]))

        capabilities = data.get("capabilities") or ["navigate_2d", "stop", "report_pose"]
        return cls(
            agent_id=agent_id,
            embodiment=str(data.get("embodiment") or data.get("type") or "mobile_base"),
            namespace=str(data.get("namespace") or f"/{agent_id}"),
            frames=FrameSpec.from_mapping(data.get("frames"), agent_id),
            capabilities=tuple(str(item) for item in capabilities),
            authority_mode=parse_authority(data.get("authority", {}).get("mode", "autonomy")),
            command_ttl_ms=int(data.get("authority", {}).get("command_ttl_ms", 200)),
            spawn=parsed_spawn,
        )


@dataclass
class AgentState:
    agent_id: str
    embodiment_type: str
    namespace: str
    lifecycle_state: LifecycleState = LifecycleState.ACTIVE
    authority_mode: AuthorityMode = AuthorityMode.AUTONOMY
    current_task_id: str = ""
    current_task_status: TaskStatus = TaskStatus.PENDING
    current_goal: tuple[float, float, float] | None = None
    pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
    linear_velocity_x: float = 0.0
    linear_velocity_y: float = 0.0
    angular_velocity_z: float = 0.0
    battery: float = 1.0
    emergency_stopped: bool = False
    fall_detected: bool = False
    capabilities: tuple[str, ...] = ()
    behavior_state: BehaviorState = BehaviorState.IDLE

    @classmethod
    def from_spec(cls, spec: AgentSpec) -> "AgentState":
        spawn = spec.spawn or (0.0, 0.0, 0.0)
        return cls(
            agent_id=spec.agent_id,
            embodiment_type=spec.embodiment,
            namespace=spec.namespace,
            authority_mode=spec.authority_mode,
            capabilities=spec.capabilities,
            pose=spawn,
        )


class AgentRegistry:
    """In-memory source of truth for runtime agents."""

    def __init__(self) -> None:
        self._specs: dict[str, AgentSpec] = {}
        self._states: dict[str, AgentState] = {}

    def register(self, spec: AgentSpec) -> None:
        if spec.agent_id in self._specs:
            raise ValueError(f"agent '{spec.agent_id}' is already registered")
        self._specs[spec.agent_id] = spec
        self._states[spec.agent_id] = AgentState.from_spec(spec)

    def register_many(self, specs: Iterable[AgentSpec]) -> None:
        for spec in specs:
            self.register(spec)

    def get_spec(self, agent_id: str) -> AgentSpec:
        try:
            return self._specs[agent_id]
        except KeyError as exc:
            raise KeyError(f"unknown agent '{agent_id}'") from exc

    def get_state(self, agent_id: str) -> AgentState:
        try:
            return self._states[agent_id]
        except KeyError as exc:
            raise KeyError(f"unknown agent '{agent_id}'") from exc

    def list_specs(self) -> list[AgentSpec]:
        return list(self._specs.values())

    def list_states(self) -> list[AgentState]:
        return list(self._states.values())

    def assign_task(self, agent_id: str, task_id: str) -> None:
        self.get_state(agent_id).current_task_id = task_id

    def clear_task(self, agent_id: str) -> None:
        self.get_state(agent_id).current_task_id = ""

    def emergency_stop(self, agent_id: str, stopped: bool = True) -> None:
        self.get_state(agent_id).emergency_stopped = stopped
