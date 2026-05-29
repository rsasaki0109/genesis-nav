"""ROS 2 bridge boundary."""

from genesis_nav.ros.qos import (
    build_qos_profile,
    load_qos_profiles,
    qos_profile_for,
    resolve_qos_for,
)

__all__ = [
    "build_qos_profile",
    "load_qos_profiles",
    "qos_profile_for",
    "resolve_qos_for",
]


def __getattr__(name: str):  # type: ignore[no-untyped-def]
    if name in {"BridgeConfig", "RosBridge", "TeleopCommandHandler"}:
        from genesis_nav.ros import bridge

        return getattr(bridge, name)
    raise AttributeError(name)
