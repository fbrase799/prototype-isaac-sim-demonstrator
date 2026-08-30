"""EXP-01 — Headless Isaac Sim startup and GPU detection.

Launch Kit without a window, confirm an NVIDIA GPU is visible, and step the
application a few times. Requires ``common.py`` in the same directory.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "width": 1280, "height": 720})

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from common import finish, log, log_app_info, log_gpu_info  # noqa: E402

log("EXP-01: Isaac Sim headless startup")
log_gpu_info()
log_app_info()
log("SimulationApp is running with headless=True")

for index in range(5):
    simulation_app.update()
    log(f"simulation_app.update() {index + 1}/5")

log("EXP-01 complete: Isaac Sim started headless and processed updates.")
finish()
