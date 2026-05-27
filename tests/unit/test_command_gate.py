from genesis_nav.core.authority import AuthorityMode
from genesis_nav.core.command_gate import CommandGate, CommandGateConfig, RuntimeCommand


def test_rejects_stale_command() -> None:
    gate = CommandGate()
    decision = gate.evaluate(
        RuntimeCommand(agent_id="r1", linear_x=0.5, issued_at_sec=1.0, ttl_ms=100),
        now_sec=1.2,
    )

    assert not decision.accepted
    assert decision.reason == "command is stale"


def test_limits_velocity() -> None:
    gate = CommandGate(CommandGateConfig(max_linear_x=1.0, max_linear_y=0.5, max_angular_z=2.0))
    decision = gate.evaluate(
        RuntimeCommand(
            agent_id="r1",
            linear_x=3.0,
            linear_y=-2.0,
            angular_z=4.0,
            issued_at_sec=10.0,
        ),
        now_sec=10.01,
    )

    assert decision.accepted
    assert decision.command is not None
    assert decision.command.linear_x == 1.0
    assert decision.command.linear_y == -0.5
    assert decision.command.angular_z == 2.0


def test_emergency_stop_rejects_command() -> None:
    gate = CommandGate()
    gate.set_emergency_stop("r1", True)

    decision = gate.evaluate(
        RuntimeCommand(agent_id="r1", linear_x=0.5, issued_at_sec=1.0),
        now_sec=1.01,
    )

    assert not decision.accepted
    assert decision.reason == "agent is emergency stopped"


def test_higher_authority_lock_blocks_lower_authority() -> None:
    gate = CommandGate()
    gate.lock_authority("r1", AuthorityMode.TELEOP, ttl_sec=1.0, reason="operator override", now_sec=5.0)

    decision = gate.evaluate(
        RuntimeCommand(
            agent_id="r1",
            authority=AuthorityMode.AUTONOMY,
            linear_x=0.5,
            issued_at_sec=5.0,
        ),
        now_sec=5.1,
    )

    assert not decision.accepted
    assert "teleop" in decision.reason


def test_ai_cannot_publish_velocity_by_default() -> None:
    gate = CommandGate()

    decision = gate.evaluate(
        RuntimeCommand(
            agent_id="r1",
            authority=AuthorityMode.AI,
            linear_x=0.1,
            issued_at_sec=1.0,
        ),
        now_sec=1.01,
    )

    assert not decision.accepted
    assert "cannot publish velocity" in decision.reason
