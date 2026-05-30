"""Record a GIF of a scenario running on the *real* Genesis backend.

Headless: attaches a Genesis camera to the scene, drives the normal runtime
(same CommandGate / behavior loop as `gnav run --backend genesis`), and writes
one RGB frame per sim tick. Frames are assembled into an MP4/GIF afterward.

Usage (in the Genesis venv):
    python scripts/record_genesis_demo.py \
        --scenario examples/scenarios/smoke.yaml --out-dir /tmp/gsframes

Genesis-only; not imported by the core package.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image

import genesis as gs

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.runtime import Runtime
from genesis_nav.genesis.adapter import GenesisDiffDriveAdapter
from genesis_nav.genesis.world_loader import load_world_entry
from genesis_nav.observability.events import JsonlEventWriter


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="examples/scenarios/smoke.yaml")
    ap.add_argument("--out-dir", default="/tmp/gsframes")
    ap.add_argument("--max-sim-seconds", type=float, default=12.0)
    ap.add_argument("--res", type=int, nargs=2, default=(640, 480))
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    scenario = load_scenario(args.scenario)

    # Build the Genesis scene ourselves so we can attach a camera before build().
    world = load_world_entry(scenario.world)
    scene = world.build_scene(scenario.seed)  # also runs gs.init()
    cam = scene.add_camera(
        res=tuple(args.res), pos=(3.0, -2.5, 3.0), lookat=(1.0, 0.5, 0.1),
        fov=45, GUI=False,
    )

    adapters: dict[str, GenesisDiffDriveAdapter] = {}

    def adapter_factory(spec):  # noqa: ANN001
        entity = world.spawn_diff_drive(scene, spec)
        a = GenesisDiffDriveAdapter(agent_id=spec.agent_id, entity=entity)
        adapters[spec.agent_id] = a
        return a

    with JsonlEventWriter(out_dir / "events.jsonl") as events:
        runtime = Runtime.from_scenario(
            scenario, events, adapter_factory=adapter_factory
        )
        scene.build()  # after all agents + camera are added
        for task in scenario.tasks:
            runtime.submit_task(task, episode_id="demo")

        frames: list[int] = []

        def on_step(_sim_time: float) -> None:
            scene.step()
            out = cam.render()
            rgb = out[0] if isinstance(out, tuple) else out
            arr = np.asarray(rgb)[:, :, :3].astype("uint8")
            idx = len(frames)
            Image.fromarray(arr).save(out_dir / f"frame_{idx:04d}.png")
            frames.append(idx)

        runtime.run_until_idle(
            episode_id="demo", max_sim_seconds=args.max_sim_seconds,
            on_step=on_step,
        )

    print(f"WROTE {len(frames)} frames to {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
