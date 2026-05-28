from genesis_nav.fleet.reservation import ReservationManager


def test_reservation_blocks_different_requester_until_expired() -> None:
    manager = ReservationManager()
    first = manager.reserve("aisle_A", "robot_001", 1.0, now_sec=10.0)
    blocked = manager.reserve("aisle_A", "robot_002", 1.0, now_sec=10.5)
    second = manager.reserve("aisle_A", "robot_002", 1.0, now_sec=11.1)

    assert first is not None
    assert blocked is None
    assert second is not None
