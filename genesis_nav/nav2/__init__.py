"""Nav2 planner backend (`runtime.navigation.planner: nav2`).

Per the 2026-05-29 ADR "Nav2 is a planner backend behind `plan()`, not a
runtime replacement", Nav2 enters as a `Nav2Planner` that implements the same
`plan()` signature as the in-tree planners. genesis-nav remains the runtime,
arbiter, and observability owner; Nav2's global planner is *delegated* to via
the `Nav2PathService` boundary. All `rclpy` use lives in `bridge.py`.
"""

from genesis_nav.nav2.planner import (
    FakeNav2PathService,
    Nav2PathService,
    Nav2Planner,
)

__all__ = [
    "FakeNav2PathService",
    "Nav2PathService",
    "Nav2Planner",
]
