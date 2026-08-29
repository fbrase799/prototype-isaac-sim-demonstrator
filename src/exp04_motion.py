"""EXP-04 — Move a rigid cube from Python and observe it with the camera.

Warm up the renderer, apply a linear velocity, step the world, and save RGB
frames so the motion is visible in the output directory.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "width": 1280, "height": 720})

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from common import (  # noqa: E402
    add_rgb_camera,
    build_simple_scene,
    cube_state,
    log,
    output_dir,
    play_simulation,
    read_rgb,
    save_rgb,
    step_simulation,
    wait_for_rgb,
)

STEPS = 90
CAPTURE_EVERY = 15
VELOCITY = np.array([1.6, 0.0, 0.0])

log("EXP-04: basic motion")
cube = build_simple_scene(cube_position=(-1.0, 0.0, 0.4))
sensor = add_rgb_camera(position=(3.2, 2.4, 1.7), target=(0.0, 0.0, 0.3))
play_simulation()
wait_for_rgb(sensor)

cube.set_velocities(linear_velocities=np.array([VELOCITY], dtype=float))
log(f"Applied linear velocity {VELOCITY.tolist()}")

out = output_dir()
captures = 0
for index in range(1, STEPS + 1):
    step_simulation()
    if index != 1 and index % CAPTURE_EVERY != 0 and index != STEPS:
        continue
    position, velocity = cube_state(cube)
    log(
        f"step={index:04d} pos={np.array2string(position, precision=3)} "
        f"vel={np.array2string(velocity, precision=3)}"
    )
    rgb = read_rgb(sensor)
    if rgb is None:
        rgb = wait_for_rgb(sensor, max_warmup_steps=8)
    captures += 1
    save_rgb(rgb, out / f"exp04_frame_{captures:02d}.png")

log(f"EXP-04 complete: wrote {captures} frames under {out}")
simulation_app.close()
