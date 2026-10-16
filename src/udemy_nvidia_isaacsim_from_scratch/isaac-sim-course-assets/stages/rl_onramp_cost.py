# 11.7: Isaac Lab -- what RL adds, and what it costs.
#
# THE HONEST CONSTRAINT, stated first. Isaac Lab is not in this container and
# is not on PyPI -- it is a separate repository install with its own toolchain.
# So this lecture cannot demo it, and inventing a demo would be exactly the
# failure this course keeps warning about.
#
# WHAT IT CAN DO, and what the curriculum actually asks for ("honest about 6 GB
# and 80 W"), is measure the thing that decides whether RL is viable on this
# hardware at all: THROUGHPUT. Reinforcement learning is not expensive because
# the maths is hard; it is expensive because it needs millions of environment
# steps, and every one of those steps costs what this script measures.
#
# WHAT IS MEASURED, on the capstone's own robot and scene:
#   1. steps per second for ONE environment, headless -- the number every
#      sample-count estimate is divided by
#   2. how that scales when the same process steps N robots, because parallel
#      environments are the entire reason GPU RL is affordable
#   3. VRAM per environment, measured on a settled stage (7.6's method), and
#      therefore how many fit in the 6141 MiB this card actually has
#   4. the arithmetic that follows: at this rate, how long is a "small" RL run
#
# The point is not to discourage anyone. It is to replace "you need a big GPU"
# with a number the learner can check on their own machine.
from isaacsim import SimulationApp

sim = SimulationApp({"headless": True})

import subprocess
import time

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage, open_stage
from isaacsim.core.prims import Articulation
from isaacsim.storage.native import get_assets_root_path

CARTER = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
          "/Assets/Isaac/4.5/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd")


def show(l=""):
    print("RLC> " + l, flush=True)


def vram():
    try:
        return int(subprocess.check_output(
            ["nvidia-smi", "--query-gpu=memory.used",
             "--format=csv,noheader,nounits"], text=True).strip().split("\n")[0])
    except Exception:
        return -1


def total_vram():
    try:
        return int(subprocess.check_output(
            ["nvidia-smi", "--query-gpu=memory.total",
             "--format=csv,noheader,nounits"], text=True).strip().split("\n")[0])
    except Exception:
        return 6141


show("=" * 74)
show("11.7  WHAT RL COSTS ON THIS HARDWARE -- measured, not asserted")
show("=" * 74)
show()
show("  ISAAC LAB IS NOT INSTALLED HERE and is not available from PyPI -- it")
show("  is a separate repository install. So this lecture does not demo it.")
show("  What it measures instead is the number that decides whether RL is")
show("  viable on this machine at all: how fast the environment steps.")
show()

root = get_assets_root_path()
open_stage(root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd")
world = World(stage_units_in_meters=1.0)

TOTAL = total_vram()
base_v = vram()
show(f"  card: {TOTAL} MiB total, {base_v} MiB in use before any robot")
show()

# ---------------------------------------------------------------------------
# 1-2. THROUGHPUT, ONE ROBOT AND THEN MANY
# ---------------------------------------------------------------------------
show("-" * 74)
show("1. STEPS PER SECOND, AND HOW IT SCALES")
show("-" * 74)
rows = []
for n in (1, 2, 4, 8):
    # add robots up to n
    for i in range(len(rows) and 0 or 0, 0):
        pass
    for i in range(n):
        path = f"/World/carter_{i}"
        st = world.stage if hasattr(world, "stage") else None
        add_reference_to_stage(usd_path=CARTER, prim_path=path)
    robots = Articulation(prim_paths_expr="/World/carter_.*", name=f"pack{n}")
    world.scene.add(robots)
    world.reset()
    # warm-up, discarded (8.2's lesson: the first steps pay for allocation)
    for _ in range(20):
        world.step(render=False)
    v_after = vram()
    t0 = time.perf_counter()
    STEPS = 200
    for _ in range(STEPS):
        world.step(render=False)
    dt = time.perf_counter() - t0
    sps = STEPS / dt
    env_sps = sps * n
    rows.append((n, sps, env_sps, v_after))
    show(f"  {n:2d} robot(s): {sps:7.1f} steps/s   "
         f"{env_sps:8.1f} env-steps/s   {v_after:5d} MiB")
    # tear down for the next count
    world.scene.clear()
    world.clear()
    open_stage(root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd")
    world = World(stage_units_in_meters=1.0)
show()

one = rows[0]
best = max(rows, key=lambda r: r[2])
show(f"  one environment runs at {one[1]:.0f} steps/s.")
show(f"  the best total here is {best[2]:.0f} env-steps/s at {best[0]} robots"
     f"  ({best[2] / one[1]:.1f}x one).")
show()
if best[2] < one[1] * best[0] * 0.8:
    show("  NOTE THE SUBLINEARITY. Doubling the robots does not double the")
    show("  throughput, because one physics scene is being stepped and the")
    show("  work per step grows. Isaac Lab's answer to that is tensorised")
    show("  environments on the GPU rather than N articulations in one scene,")
    show("  which is a different architecture, not a bigger number.")
    show()

# ---------------------------------------------------------------------------
# 3. THE ARITHMETIC THAT DECIDES EVERYTHING
# ---------------------------------------------------------------------------
show("-" * 74)
show("2. WHAT THAT MEANS FOR A TRAINING RUN")
show("-" * 74)
rate = best[2]
for steps in (1_000_000, 10_000_000, 100_000_000):
    hrs = steps / rate / 3600
    show(f"  {steps:>12,} env-steps  ->  {hrs:8.1f} hours at {rate:.0f}/s")
show()
show("  Those are the real numbers for this laptop. A locomotion policy in")
show("  the literature is commonly tens to hundreds of millions of steps, so")
show("  the honest reading is that this hardware trains SMALL policies on")
show("  SIMPLE tasks, overnight, and does not reproduce a paper.")
show()
show("  That is not a reason to avoid RL. It is a reason to scope the first")
show("  attempt to something that fits, which is exactly what 11.8 does.")
show()

# ---------------------------------------------------------------------------
# 4. THE OTHER CEILING
# ---------------------------------------------------------------------------
show("-" * 74)
show("3. THE OTHER CEILING: MEMORY, AND POWER")
show("-" * 74)
per_env = (rows[-1][3] - base_v) / rows[-1][0] if rows[-1][0] else 0
show(f"  measured at {rows[-1][0]} robots: {rows[-1][3]} MiB used, "
     f"{base_v} MiB baseline")
show(f"  so roughly {per_env:.0f} MiB per environment")
if per_env > 0:
    head = (TOTAL - base_v) / per_env
    show(f"  which puts a ceiling near {head:.0f} environments on a "
         f"{TOTAL} MiB card")
show()
show("  And the number nobody quotes: this is an 80 W laptop GPU. Sustained")
show("  RL training runs it at full load for hours, which means thermal")
show("  throttling is part of your throughput budget rather than a footnote.")
show("  Measure your steps per second at the START of a run and again an hour")
show("  in, because the second number is the one your estimate should use.")
show()
show("=" * 74)
show("done")
show("=" * 74)

sim.close()
