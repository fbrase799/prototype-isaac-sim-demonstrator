"""EXP-02 — Simple scene with a ground plane and one rigid cube.

Create a PhysicsScene, drop a dynamic cube, play the timeline, and step for a
fixed number of frames while printing pose and velocity.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "width": 1280, "height": 720})

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from common import build_simple_scene, finish, log, play_simulation, step_and_report  # noqa: E402

STEPS = 120

log("EXP-02: simple scene")
cube = build_simple_scene()
play_simulation()
log("Timeline playing; stepping physics.")
step_and_report(cube, steps=STEPS, report_every=30)
log("EXP-02 complete.")
finish()
