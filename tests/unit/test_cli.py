import json
from pathlib import Path

from genesis_nav.cli.main import main


def test_run_writes_replay_artifacts(tmp_path: Path) -> None:
    code = main(
        [
            "run",
            "examples/scenarios/smoke.yaml",
            "--fast",
            "--record",
            "--output-dir",
            str(tmp_path),
        ]
    )

    assert code == 0
    run_dirs = list(tmp_path.iterdir())
    assert len(run_dirs) == 1
    run_dir = run_dirs[0]
    assert (run_dir / "scenario.yaml").exists()
    assert (run_dir / "resolved_config.yaml").exists()
    assert (run_dir / "events.jsonl").exists()
    assert (run_dir / "metrics.json").exists()
    assert (run_dir / "report.md").exists()
    assert (run_dir / "rosbag").is_dir()

    metrics = json.loads((run_dir / "metrics.json").read_text(encoding="utf-8"))
    assert metrics["scenario_id"] == "smoke"
    assert metrics["success_rate"] == 1.0


def test_replay_validates_artifacts(tmp_path: Path) -> None:
    main(
        [
            "run",
            "examples/scenarios/smoke.yaml",
            "--fast",
            "--output-dir",
            str(tmp_path),
        ]
    )
    run_dir = next(tmp_path.iterdir())

    assert main(["replay", str(run_dir)]) == 0
