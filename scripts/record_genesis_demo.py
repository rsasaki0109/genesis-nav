"""Record a GIF of a scenario running on the *real* Genesis backend.

Headless and cinematic: builds the Genesis scene with a colored robot body, a
goal marker, a chase camera that follows the robot, and a growing trajectory
trail — then drives the normal runtime (same CommandGate / behavior loop as
`gnav run --backend genesis`) and writes one RGB frame per sim tick. Frames are
assembled into a GIF/MP4 afterward with ffmpeg.

Usage (in a Genesis venv):
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
from genesis_nav.observability.events import JsonlEventWriter

ROBOT_COLOR = (0.20, 0.45, 0.95)   # blue
GOAL_COLOR = (0.15, 0.85, 0.30)    # green
TRAIL_COLOR = (1.0, 0.55, 0.05, 1.0)  # orange


def _goal_of(scenario):  # noqa: ANN001
    for task in scenario.tasks:
        g = getattr(task, "goal", None)
        if g is not None:
            return (float(g[0]), float(g[1]), float(g[2]))
    return (0.0, 0.0, 0.0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="examples/scenarios/smoke.yaml")
    ap.add_argument("--out-dir", default="/tmp/gsframes")
    ap.add_argument("--max-sim-seconds", type=float, default=12.0)
    ap.add_argument("--res", type=int, nargs=2, default=(720, 540))
    ap.add_argument("--trail-dots", type=int, default=120)
    ap.add_argument("--trail-spacing-m", type=float, default=0.05)
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    scenario = load_scenario(args.scenario)
    goal = _goal_of(scenario)

    # Build the scene here so we can style entities + attach the chase camera.
    gs.init(backend=getattr(gs, "gpu", None) or gs.cpu, seed=scenario.seed)
    scene = gs.Scene(show_viewer=False)
    scene.add_entity(gs.morphs.Plane())

    # Goal marker: a green disc at the goal. collision=False so it never blocks
    # the robot (a colliding marker stalls it short of the goal tolerance).
    scene.add_entity(
        gs.morphs.Cylinder(
            radius=0.18, height=0.04, pos=(goal[0], goal[1], 0.02),
            fixed=True, collision=False,
        ),
        surface=gs.surfaces.Default(color=GOAL_COLOR),
    )

    # Trajectory trail: a pool of small collision-free discs, parked below the
    # ground and repositioned along the path as the robot moves. Genesis debug
    # lines are viewer-only (they do not appear in an offscreen camera render),
    # so the trail must be real scene geometry.
    trail_pool = [
        scene.add_entity(
            gs.morphs.Cylinder(
                radius=0.05, height=0.02, pos=(0.0, 0.0, -10.0),
                fixed=True, collision=False,
            ),
            surface=gs.surfaces.Default(color=TRAIL_COLOR[:3]),
        )
        for _ in range(args.trail_dots)
    ]

    # Robots: a colored box per agent. Wrap each in the real Genesis adapter.
    adapters: dict[str, GenesisDiffDriveAdapter] = {}
    for spec in scenario.agents:
        spawn = spec.spawn or (0.0, 0.0, 0.0)
        box = scene.add_entity(
            gs.morphs.Box(size=(0.4, 0.4, 0.2), pos=(spawn[0], spawn[1], 0.1)),
            surface=gs.surfaces.Default(color=ROBOT_COLOR),
        )
        adapters[spec.agent_id] = GenesisDiffDriveAdapter(agent_id=spec.agent_id, entity=box)

    cam = scene.add_camera(res=tuple(args.res), pos=(3.0, -3.0, 3.0),
                           lookat=(goal[0] * 0.5, goal[1] * 0.5, 0.1), fov=42, GUI=False)
    scene.build()

    primary = scenario.agents[0].agent_id

    with JsonlEventWriter(out_dir / "events.jsonl") as events:
        runtime = Runtime.from_scenario(
            scenario, events, adapter_factory=lambda spec: adapters[spec.agent_id]
        )
        for task in scenario.tasks:
            runtime.submit_task(task, episode_id="demo")

        frames: list[int] = []
        trail_used = 0
        last_trail_xy: tuple[float, float] | None = None

        def on_step(_sim_time: float) -> None:
            nonlocal trail_used, last_trail_xy
            scene.step()
            # Drop a trail dot behind the primary agent every trail_spacing_m.
            px, py, _ = adapters[primary].read_pose()
            if last_trail_xy is None or (
                (px - last_trail_xy[0]) ** 2 + (py - last_trail_xy[1]) ** 2
            ) >= args.trail_spacing_m ** 2:
                if trail_used < len(trail_pool):
                    trail_pool[trail_used].set_pos([px, py, 0.03])
                    trail_used += 1
                    last_trail_xy = (px, py)
            # Chase camera: trail the primary agent at a 3/4 angle.
            cam.set_pose(pos=(px - 2.2, py - 2.2, 1.9), lookat=(px, py, 0.2))
            out = cam.render()
            rgb = out[0] if isinstance(out, tuple) else out
            arr = np.asarray(rgb)[:, :, :3].astype("uint8")
            idx = len(frames)
            Image.fromarray(arr).save(out_dir / f"frame_{idx:04d}.png")
            frames.append(idx)

        runtime.run_until_idle(
            episode_id="demo", max_sim_seconds=args.max_sim_seconds, on_step=on_step
        )

    print(f"WROTE {len(frames)} frames to {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
