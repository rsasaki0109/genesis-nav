"""Resource model for fleet coordination.

Resources describe shared zones (narrow aisles, charging docks, intersections)
that the runtime treats as exclusive leases. The schema is intentionally tiny
in v0.1; richer geometry can be added without changing the lease API.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable


@dataclass(frozen=True)
class Resource:
    resource_id: str
    kind: str = "zone"
    capacity: int = 1
    metadata: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "Resource":
        resource_id = str(data.get("id") or data.get("resource_id") or "").strip()
        if not resource_id:
            raise ValueError("resource entry requires 'id'")
        return cls(
            resource_id=resource_id,
            kind=str(data.get("kind", "zone")),
            capacity=int(data.get("capacity", 1)),
            metadata={k: v for k, v in data.items() if k not in {"id", "resource_id", "kind", "capacity"}},
        )


class ResourceCatalog:
    def __init__(self, resources: Iterable[Resource] = ()) -> None:
        self._by_id: dict[str, Resource] = {res.resource_id: res for res in resources}

    @classmethod
    def from_scenario(cls, raw: dict[str, Any]) -> "ResourceCatalog":
        entries = raw.get("resources") or []
        if not isinstance(entries, list):
            raise ValueError("scenario 'resources' must be a list")
        return cls(Resource.from_mapping(entry) for entry in entries)

    def __len__(self) -> int:
        return len(self._by_id)

    def __iter__(self):
        return iter(self._by_id.values())

    def __contains__(self, resource_id: str) -> bool:
        return resource_id in self._by_id

    def get(self, resource_id: str) -> Resource | None:
        return self._by_id.get(resource_id)

    def ids(self) -> list[str]:
        return list(self._by_id)
