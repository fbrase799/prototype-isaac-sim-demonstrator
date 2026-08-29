"""EXP-03 — Simulated camera RGB capture.

Add an RTX camera looking at the demo cube, wait for the renderer to produce a
frame, and write a PNG under the output directory.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "width": 1280, "height": 720})

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from common import (  # noqa: E402
    add_rgb_camera,
    build_simple_scene,
    log,
    output_dir,
    play_simulation,
    save_rgb,
    wait_for_rgb,
)

log("EXP-03: camera RGB capture")
build_simple_scene()
sensor = add_rgb_camera()
play_simulation()

rgb = wait_for_rgb(sensor)
image_path = output_dir() / "exp03_rgb.png"
save_rgb(rgb, image_path)
log(f"EXP-03 complete: {image_path}")
simulation_app.close()
