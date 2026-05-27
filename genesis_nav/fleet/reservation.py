"""Lease-based resource reservation foundation."""

from __future__ import annotations

from dataclasses import dataclass
from time import monotonic
from uuid import uuid4


@dataclass(frozen=True)
class ResourceLease:
    lease_id: str
    resource_id: str
    requester_id: str
    expires_at_sec: float


class ReservationManager:
    def __init__(self) -> None:
        self._leases: dict[str, ResourceLease] = {}

    def reserve(
        self,
        resource_id: str,
        requester_id: str,
        duration_sec: float,
        *,
        now_sec: float | None = None,
    ) -> ResourceLease | None:
        if duration_sec <= 0:
            raise ValueError("duration_sec must be positive")
        now = monotonic() if now_sec is None else now_sec
        self._expire(now)
        current = self._leases.get(resource_id)
        if current and current.requester_id != requester_id:
            return None
        lease = ResourceLease(
            lease_id=str(uuid4()),
            resource_id=resource_id,
            requester_id=requester_id,
            expires_at_sec=now + duration_sec,
        )
        self._leases[resource_id] = lease
        return lease

    def release(self, lease_id: str) -> bool:
        for resource_id, lease in list(self._leases.items()):
            if lease.lease_id == lease_id:
                self._leases.pop(resource_id)
                return True
        return False

    def _expire(self, now_sec: float) -> None:
        for resource_id, lease in list(self._leases.items()):
            if lease.expires_at_sec <= now_sec:
                self._leases.pop(resource_id)
