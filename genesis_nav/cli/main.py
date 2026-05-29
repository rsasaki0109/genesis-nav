"""`gnav` command-line entry point."""

from __future__ import annotations

import argparse
import importlib.util
import json
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from genesis_nav import __version__
from genesis_nav.benchmarks.report import (
    BenchmarkExpectation,
    BenchmarkScenarioResult,
    BenchmarkSuiteReport,
    discover_scenarios,
    now_iso,
)
from genesis_nav.benchmarks.scenario import Scenario, load_scenario
from genesis_nav.core.runtime import Runtime, ensure_run_layout
from genesis_nav.navigation.config import NavigationConfig
from genesis_nav.observability.env import collect_env_metadata, write_env_metadata
from genesis_nav.observability.events import (
    FanoutEventSink,
    JsonlEventWriter,
    RingBufferEventSink,
)
from genesis_nav.observability.metrics import MetricsSnapshot


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="gnav")
    parser.add_argument("--version", action="version", version=f"gnav {__version__}")
    subparsers = parser.add_subparsers(dest="command", required=True)

    run = subparsers.add_parser("run", help="run a reproducible scenario")
    run.add_argument("scenario", type=Path)
    run.add_argument("--fast", action="store_true", help="run without wall-time synchronization")
    run.add_argument("--record", action="store_true", help="create rosbag artifact directory")
    run.add_argument("--output-dir", type=Path, default=Path("runs"))
    run.add_argument(
        "--ros",
        action="store_true",
        help="bridge runtime state to a ROS 2 graph (requires rclpy and genesis_nav_msgs)",
    )
    run.add_argument(
        "--qos-profile",
        type=Path,
        default=Path("configs/qos/default.yaml"),
        help="QoS profile YAML used by --ros",
    )
    run.add_argument(
        "--backend",
        choices=("fallback", "genesis", "ros2_robot"),
        default="fallback",
        help="embodiment backend: 'fallback' uses the in-memory diff-drive, "
        "'genesis' uses the Genesis adapter via the scenario world file, "
        "'ros2_robot' drives a real robot over /<agent>/cmd_vel + /<agent>/odom "
        "(requires a sourced ROS 2 environment)",
    )
    run.set_defaults(func=run_command)

    replay = subparsers.add_parser("replay", help="validate a run artifact for replay")
    replay.add_argument("run_dir", type=Path)
    replay.add_argument(
        "--print-events",
        action="store_true",
        help="stream task and safety events from events.jsonl in order",
    )
    replay.set_defaults(func=replay_command)

    bench = subparsers.add_parser("bench", help="validate or run benchmark scenarios")
    bench.add_argument(
        "scenario",
        type=Path,
        nargs="?",
        help="benchmark scenario file (omitted with --run)",
    )
    bench.add_argument(
        "--run",
        type=Path,
        metavar="SUITE_DIR",
        help="run every *.yaml under SUITE_DIR and emit a benchmark report",
    )
    bench.add_argument(
        "--output-dir",
        type=Path,
        default=Path("benchmarks/_runs"),
        help="root directory for benchmark run artifacts (default: benchmarks/_runs)",
    )
    bench.add_argument(
        "--report",
        type=Path,
        default=None,
        help="path to write the benchmark report JSON (default: <output-dir>/<suite>_report.json)",
    )
    bench.set_defaults(func=bench_command)

    doctor = subparsers.add_parser("doctor", help="check local runtime dependencies")
    doctor.set_defaults(func=doctor_command)

    return parser


def run_command(args: argparse.Namespace) -> int:
    scenario = load_scenario(args.scenario)
    run_dir = create_run_dir(args.output_dir, scenario)
    record_rosbag = bool(args.record or scenario.record.rosbag)
    ensure_run_layout(run_dir, record_rosbag=record_rosbag)

    shutil.copyfile(scenario.source_path, run_dir / "scenario.yaml")
    write_yaml(run_dir / "resolved_config.yaml", scenario.raw)

    episode_id = f"{run_dir.name}_seed{scenario.seed}"
    max_sim_seconds = float(scenario.raw.get("max_sim_seconds", 60.0))
    mode = "fast" if args.fast else "realtime"
    planner_choice = NavigationConfig.from_scenario_raw(scenario.raw).planner

    env_metadata = collect_env_metadata(
        scenario_id=scenario.scenario_id,
        seed=scenario.seed,
        backend=args.backend,
        mode=mode,
        ros_enabled=bool(args.ros),
        record_rosbag=record_rosbag,
        planner=planner_choice,
    )
    write_env_metadata(run_dir / "env.json", env_metadata)

    if args.ros:
        qos_source = Path(args.qos_profile)
        if qos_source.exists():
            shutil.copyfile(qos_source, run_dir / "qos_profile.yaml")

    bridge = None
    genesis_backend = None
    robot_backend = None
    if args.backend == "genesis":
        try:
            from genesis_nav.genesis.backend import (
                GenesisNotAvailableError,
                build_genesis_backend,
            )
            genesis_backend = build_genesis_backend(scenario)
        except GenesisNotAvailableError as exc:
            print(f"--backend genesis unavailable: {exc}", file=sys.stderr)
            return 4
    elif args.backend == "ros2_robot":
        try:
            from genesis_nav.ros2_robot.backend import (
                Ros2RobotNotAvailableError,
                build_ros2_robot_backend,
            )
            robot_backend = build_ros2_robot_backend(scenario)
        except Ros2RobotNotAvailableError as exc:
            print(f"--backend ros2_robot unavailable: {exc}", file=sys.stderr)
            return 4

    with JsonlEventWriter(run_dir / "events.jsonl") as jsonl_sink:
        event_buffer = RingBufferEventSink(capacity=2048)
        event_sink: Any = FanoutEventSink([jsonl_sink, event_buffer])
        adapter_factory = None
        if genesis_backend is not None:
            adapter_factory = genesis_backend.spawn
        elif robot_backend is not None:
            adapter_factory = robot_backend.spawn
        from genesis_nav.nav2.bridge import Nav2NotAvailableError

        try:
            runtime = Runtime.from_scenario(
                scenario, event_sink, adapter_factory=adapter_factory
            )
        except Nav2NotAvailableError as exc:
            print(
                f"runtime.navigation.planner: nav2 unavailable: {exc}",
                file=sys.stderr,
            )
            return 4
        tool_api = runtime.tool_api(
            scenario_id=scenario.scenario_id, event_buffer=event_buffer
        )
        del tool_api  # exposed for downstream embedders; not used inside `gnav run`

        if args.ros:
            try:
                from genesis_nav.ros.bridge import BridgeConfig, RosBridge
            except ImportError as exc:
                print(f"--ros requires rclpy and genesis_nav_msgs: {exc}", file=sys.stderr)
                return 3
            bridge = RosBridge(
                runtime.registry,
                runtime.command_gate,
                runtime.clock,
                jsonl_sink,
                config=BridgeConfig(qos_path=args.qos_profile),
                external_command_handler=runtime.apply_external_command,
                episode_id=episode_id,
            )
            event_sink = FanoutEventSink([jsonl_sink, event_buffer, bridge])
            runtime.events = event_sink

        try:
            event_sink.write(
                ts=0.0,
                episode_id=episode_id,
                event="SCENARIO_STARTED",
                data={
                    "scenario_id": scenario.scenario_id,
                    "seed": scenario.seed,
                    "mode": mode,
                    "agent_count": len(scenario.agents),
                    "task_count": len(scenario.tasks),
                },
            )
            for task in scenario.tasks:
                runtime.submit_task(task, episode_id=episode_id)
            runtime.dispatch_pending(episode_id=episode_id)

            on_step = None
            if bridge is not None:
                bridge.publish_clock(0.0)
                bridge.publish_states(0.0)
                bridge.publish_scenario_state(
                    0.0,
                    scenario_id=scenario.scenario_id,
                    seed=scenario.seed,
                    runtime_mode=mode,
                    paused=False,
                    recording=record_rosbag,
                )

            def on_step(sim_time: float) -> None:
                if genesis_backend is not None:
                    genesis_backend.step(runtime.clock.step_sec)
                if robot_backend is not None:
                    robot_backend.step(runtime.clock.step_sec)
                if bridge is not None:
                    bridge.publish_clock(sim_time)
                    bridge.publish_states(sim_time)
                    snapshot = runtime.metrics.summary()
                    bridge.publish_fleet_state(
                        sim_time,
                        pending=len(runtime.task_queue),
                        active=sum(
                            1
                            for state in runtime.registry.list_states()
                            if state.current_task_id
                        ),
                        completed=int(snapshot["task_succeeded_count"]),
                    )
                    bridge.spin_once(timeout_sec=0.0)

            runtime.run_until_idle(
                episode_id=episode_id,
                max_sim_seconds=max_sim_seconds,
                on_step=on_step,
            )
            summary = runtime.metrics.summary()
            event_sink.write(
                ts=runtime.clock.sim_time_sec,
                episode_id=episode_id,
                event="SCENARIO_FINISHED",
                data={"task_count": len(scenario.tasks), "summary": summary},
            )
            if bridge is not None:
                bridge.publish_fleet_state(
                    runtime.clock.sim_time_sec,
                    completed=int(summary["task_succeeded_count"]),
                    extra={"summary": summary},
                )
        finally:
            if bridge is not None:
                bridge.shutdown()
            if robot_backend is not None:
                robot_backend.shutdown()

    metrics = MetricsSnapshot(
        scenario_id=scenario.scenario_id,
        seed=scenario.seed,
        agent_count=len(scenario.agents),
        task_count=len(scenario.tasks),
        success_rate=float(summary["success_rate"]),
        task_succeeded_count=int(summary["task_succeeded_count"]),
        task_failed_count=int(summary["task_failed_count"]),
        command_accept_count=int(summary["command_accept_count"]),
        command_rejection_count=int(summary["command_rejection_count"]),
        time_to_goal_mean_sec=float(summary["time_to_goal_mean_sec"]),
        path_length_mean_m=float(summary["path_length_mean_m"]),
        sim_steps=int(summary["sim_steps"]),
        task_dispatched_count=int(summary["task_dispatched_count"]),
        task_pending_peak=int(summary["task_pending_peak"]),
        reservation_granted_count=int(summary["reservation_granted_count"]),
        reservation_conflict_count=int(summary["reservation_conflict_count"]),
        reservation_released_count=int(summary["reservation_released_count"]),
        replan_count=int(summary["replan_count"]),
        obstacle_event_count=int(summary["obstacle_event_count"]),
    )
    write_json(run_dir / "metrics.json", metrics.to_dict())
    write_report(run_dir / "report.md", scenario, metrics)

    print(run_dir)
    return 0


REPLAY_REQUIRED_ARTIFACTS = (
    "scenario.yaml",
    "resolved_config.yaml",
    "events.jsonl",
    "metrics.json",
    "report.md",
    "env.json",
)

REPLAY_REQUIRED_METRICS = (
    "scenario_id",
    "seed",
    "success_rate",
    "sim_steps",
)

REPLAY_PLAYBACK_EVENTS = frozenset(
    {
        "SCENARIO_STARTED",
        "SCENARIO_FINISHED",
        "TASK_ASSIGNED",
        "TASK_STARTED",
        "TASK_SUCCEEDED",
        "TASK_FAILED",
        "SAFETY_STOP",
        "FALL_DETECTED",
        "COLLISION",
        "AGENT_STUCK",
    }
)


def replay_command(args: argparse.Namespace) -> int:
    run_dir = args.run_dir
    missing = [name for name in REPLAY_REQUIRED_ARTIFACTS if not (run_dir / name).exists()]
    if missing:
        print(f"missing replay artifacts: {', '.join(missing)}", file=sys.stderr)
        return 2

    events_path = run_dir / "events.jsonl"
    try:
        records = _load_events(events_path)
    except _ReplayValidationError as exc:
        print(f"invalid events.jsonl: {exc}", file=sys.stderr)
        return 2

    if not records:
        print("events.jsonl is empty", file=sys.stderr)
        return 2
    if records[0]["event"] != "SCENARIO_STARTED":
        print("first event must be SCENARIO_STARTED", file=sys.stderr)
        return 2
    if records[-1]["event"] != "SCENARIO_FINISHED":
        print("last event must be SCENARIO_FINISHED", file=sys.stderr)
        return 2

    try:
        metrics = json.loads((run_dir / "metrics.json").read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        print(f"invalid metrics.json: {exc}", file=sys.stderr)
        return 2
    metrics_missing = [key for key in REPLAY_REQUIRED_METRICS if key not in metrics]
    if metrics_missing:
        print(
            f"metrics.json missing keys: {', '.join(metrics_missing)}",
            file=sys.stderr,
        )
        return 2

    if args.print_events:
        for record in records:
            if record["event"] not in REPLAY_PLAYBACK_EVENTS:
                continue
            print(
                f"{record['ts']:>10.3f}  {record['event']:<18}  "
                f"agent={record.get('agent_id', '') or '-':<14}  "
                f"task={record.get('task_id', '') or '-'}"
            )

    print(f"replay artifacts valid: {run_dir}")
    return 0


class _ReplayValidationError(Exception):
    """Raised when events.jsonl cannot be parsed for replay validation."""


def _load_events(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_no, raw in enumerate(handle, start=1):
            stripped = raw.strip()
            if not stripped:
                continue
            try:
                record = json.loads(stripped)
            except json.JSONDecodeError as exc:
                raise _ReplayValidationError(f"line {line_no}: {exc}") from exc
            for key in ("ts", "episode_id", "event"):
                if key not in record:
                    raise _ReplayValidationError(
                        f"line {line_no}: missing field '{key}'"
                    )
            records.append(record)
    return records


def bench_command(args: argparse.Namespace) -> int:
    if args.run is not None:
        return _bench_run_suite(args)
    if args.scenario is None:
        print("bench: provide a scenario path or --run <suite_dir>", file=sys.stderr)
        return 2
    scenario = load_scenario(args.scenario)
    print(
        json.dumps(
            {
                "scenario_id": scenario.scenario_id,
                "seed": scenario.seed,
                "agents": len(scenario.agents),
                "tasks": len(scenario.tasks),
                "metrics": list(scenario.metrics),
            },
            sort_keys=True,
        )
    )
    return 0


def _bench_run_suite(args: argparse.Namespace) -> int:
    suite_dir: Path = args.run
    try:
        scenarios = discover_scenarios(suite_dir)
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if not scenarios:
        print(f"no benchmark scenarios found under {suite_dir}", file=sys.stderr)
        return 2

    suite_name = suite_dir.name
    args.output_dir.mkdir(parents=True, exist_ok=True)
    suite_runs_dir = args.output_dir / suite_name
    suite_runs_dir.mkdir(parents=True, exist_ok=True)

    results: list[BenchmarkScenarioResult] = []
    for scenario_path in scenarios:
        scenario = load_scenario(scenario_path)
        try:
            expectation = BenchmarkExpectation.from_scenario_raw(scenario.raw)
        except ValueError as exc:
            print(f"{scenario_path}: {exc}", file=sys.stderr)
            return 2

        rc = main(
            [
                "run",
                str(scenario_path),
                "--fast",
                "--output-dir",
                str(suite_runs_dir),
            ]
        )
        if rc != 0:
            results.append(
                BenchmarkScenarioResult(
                    scenario_id=scenario.scenario_id,
                    scenario_path=str(scenario_path),
                    seed=scenario.seed,
                    run_dir="",
                    passed=False,
                    failures=[f"gnav run exited with code {rc}"],
                    expected=dict(expectation.raw),
                )
            )
            continue

        actual_run_dir = _pick_latest_run_dir(suite_runs_dir, scenario)
        metrics = _read_metrics(actual_run_dir)
        failures = expectation.evaluate(metrics)
        results.append(
            BenchmarkScenarioResult(
                scenario_id=scenario.scenario_id,
                scenario_path=str(scenario_path),
                seed=scenario.seed,
                run_dir=str(actual_run_dir),
                passed=not failures,
                failures=failures,
                metrics=metrics,
                expected=dict(expectation.raw),
            )
        )

    report = BenchmarkSuiteReport(
        benchmark_suite=suite_name,
        ran_at=now_iso(),
        scenarios=results,
    )
    report_path = args.report or (args.output_dir / f"{suite_name}_report.json")
    write_json(report_path, report.to_dict())
    print(
        f"benchmark suite '{suite_name}': "
        f"{report.passed}/{report.total} passed -> {report_path}"
    )
    for result in results:
        if not result.passed:
            print(
                f"  FAIL {result.scenario_id}: {'; '.join(result.failures)} "
                f"(run_dir={result.run_dir or 'n/a'})",
                file=sys.stderr,
            )
    return 0 if report.failed == 0 else 1


def _pick_latest_run_dir(suite_runs_dir: Path, scenario: Scenario) -> Path:
    candidates = sorted(
        p
        for p in suite_runs_dir.iterdir()
        if p.is_dir() and p.name.endswith(f"_{scenario.scenario_id}_seed{scenario.seed}")
    )
    if not candidates:
        raise FileNotFoundError(
            f"could not locate run directory for {scenario.scenario_id}"
        )
    return candidates[-1]


def _read_metrics(run_dir: Path) -> dict[str, Any]:
    path = run_dir / "metrics.json"
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def doctor_command(args: argparse.Namespace) -> int:
    del args
    checks: list[tuple[str, bool, str]] = [
        ("python", True, ""),
        ("yaml", True, ""),
        (
            "rclpy",
            importlib.util.find_spec("rclpy") is not None,
            "install ROS 2 (jazzy/humble) and `source /opt/ros/<distro>/setup.bash` before --ros",
        ),
        (
            "genesis_nav_msgs",
            importlib.util.find_spec("genesis_nav_msgs") is not None,
            "colcon build --base-paths ros2_ws/src --packages-select genesis_nav_msgs",
        ),
        (
            "genesis",
            importlib.util.find_spec("genesis") is not None,
            "pip install genesis-world  # required for --backend genesis",
        ),
    ]
    for name, ok, hint in checks:
        status = "ok" if ok else "missing"
        line = f"{name}: {status}"
        if not ok and hint:
            line += f"  ({hint})"
        print(line)
    return 0


def create_run_dir(root: Path, scenario: Scenario) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    base = root / f"{stamp}_{scenario.scenario_id}_seed{scenario.seed}"
    candidate = base
    suffix = 1
    while candidate.exists():
        suffix += 1
        candidate = root / f"{base.name}_{suffix}"
    return candidate


def write_yaml(path: Path, data: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, sort_keys=True)


def write_json(path: Path, data: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, sort_keys=True)
        handle.write("\n")


def write_report(path: Path, scenario: Scenario, metrics: MetricsSnapshot) -> None:
    lines = [
        f"# Run Report: {scenario.scenario_id}",
        "",
        f"- seed: `{scenario.seed}`",
        f"- world: `{scenario.world}`",
        f"- agents: `{len(scenario.agents)}`",
        f"- tasks: `{len(scenario.tasks)}`",
        f"- success_rate: `{metrics.success_rate}`",
        f"- tasks_succeeded: `{metrics.task_succeeded_count}`",
        f"- tasks_failed: `{metrics.task_failed_count}`",
        f"- time_to_goal_mean_sec: `{metrics.time_to_goal_mean_sec:.3f}`",
        f"- path_length_mean_m: `{metrics.path_length_mean_m:.3f}`",
        f"- command_accept_count: `{metrics.command_accept_count}`",
        f"- command_rejection_count: `{metrics.command_rejection_count}`",
        f"- task_dispatched_count: `{metrics.task_dispatched_count}`",
        f"- task_pending_peak: `{metrics.task_pending_peak}`",
        f"- reservation_granted_count: `{metrics.reservation_granted_count}`",
        f"- reservation_conflict_count: `{metrics.reservation_conflict_count}`",
        f"- sim_steps: `{metrics.sim_steps}`",
        "",
        "This report is generated by the v0.1 runtime loop.",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
