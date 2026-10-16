# 11.2: assembling the full robot and scene.
#
# The capstone runs. This lecture is about what it is MADE OF, and the honest
# version of that is not a tour -- it is an accounting. Every piece costs
# something to load, occupies something on the card, and was decided earlier in
# the course for a reason that can be restated in one line.
#
# WHAT IS MEASURED, in the order a learner would build it:
#   1. an empty stage                      the floor
#   2. + the warehouse                     Section 7's environment
#   3. + the robot                         Section 5's Carter
#   4. + the physics scene reset           what world.reset() actually costs
# reported as seconds, prims, and VRAM at each step, so "assembling a scene" is
# a set of numbers rather than a sequence of clicks.
#
# AND THE ORDER MATTERS, which is 7.1's lesson arriving where it bites:
# open_stage REPLACES the stage. Load the environment FIRST and add the robot
# after, or the robot silently disappears. This script demonstrates that rather
# than warning about it.
from isaacsim import SimulationApp

sim = SimulationApp({"headless": True})

import subprocess
import time

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage, open_stage
from isaacsim.core.prims import Articulation
from isaacsim.storage.native import get_assets_root_path
from pxr import Usd, UsdGeom

CARTER = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
          "/Assets/Isaac/4.5/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd")


def show(l=""):
    print("ASM> " + l, flush=True)


def vram():
    try:
        out = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
            text=True)
        return int(out.strip().split("\n")[0])
    except Exception:
        return -1


def count():
    st = omni.usd.get_context().get_stage()
    p = m = 0
    for prim in Usd.PrimRange(st.GetPseudoRoot()):
        p += 1
        if prim.IsA(UsdGeom.Mesh):
            m += 1
    return p, m


rows = []


def stamp(label, t):
    p, m = count()
    v = vram()
    rows.append((label, t, p, m, v))
    show(f"  {label:34s} {t:6.2f} s   {p:5d} prims  {m:4d} meshes  {v:5d} MiB")


show("=" * 70)
show("11.2  ASSEMBLING THE CAPSTONE -- what each piece costs")
show("=" * 70)
show()
show("-" * 70)
show("BUILT IN THE ORDER A LEARNER WOULD BUILD IT")
show("-" * 70)
stamp("empty stage", 0.0)

root = get_assets_root_path()
t0 = time.perf_counter()
open_stage(root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd")
stamp("+ the warehouse (Section 7)", time.perf_counter() - t0)

world = World(stage_units_in_meters=1.0)
t0 = time.perf_counter()
add_reference_to_stage(usd_path=CARTER, prim_path="/World/carter")
stamp("+ the robot (Section 5)", time.perf_counter() - t0)

robot = Articulation(prim_paths_expr="/World/carter", name="carter")
world.scene.add(robot)
t0 = time.perf_counter()
world.reset()
stamp("+ world.reset()", time.perf_counter() - t0)
show()

d_env_p = rows[1][2] - rows[0][2]
d_rob_p = rows[2][2] - rows[1][2]
d_env_v = rows[1][4] - rows[0][4]
d_rob_v = rows[2][4] - rows[1][4]
show("  PRIMS -- the number that is trustworthy here:")
show(f"    the warehouse adds {d_env_p:5d} prims  "
     f"({rows[0][2]} -> {rows[1][2]})")
show(f"    the robot adds     {d_rob_p:5d} prims  "
     f"({rows[1][2]} -> {rows[2][2]})")
show(f"    so the environment is {d_env_p / max(d_rob_p, 1):.1f}x the robot")
show()
# VRAM DELTAS ACROSS A STAGE LOAD ARE NOT TRUSTWORTHY AND THE SCRIPT SAYS SO.
# Measured here, adding a 3425-prim warehouse changed reported VRAM by -122
# MiB. A scene cannot cost negative memory: Kit frees the previous stage's
# allocations, warms caches, and defers uploads, so a single before/after pair
# straddling a load is measuring the allocator rather than the scene. Same
# discipline as 10.4's negative annotator timings and 8.6's -0.126 ms read.
show("  VRAM -- and why this one is NOT trustworthy:")
show(f"    warehouse {rows[0][4]} -> {rows[1][4]} MiB   ({d_env_v:+d})")
show(f"    robot     {rows[1][4]} -> {rows[2][4]} MiB   ({d_rob_v:+d})")
if d_env_v < 0:
    show()
    show(f"    Adding a {d_env_p}-prim warehouse reportedly FREED "
         f"{-d_env_v} MiB.")
    show("    A scene cannot cost negative memory. open_stage tears down the")
    show("    previous stage while building the new one, and Kit defers")
    show("    uploads and warms caches around it, so a single before/after")
    show("    pair straddling a load measures the allocator, not the scene.")
    show("    7.6 measured VRAM properly -- on a settled stage, with a stale")
    show("    process killed first -- and that is the method to use when the")
    show("    number matters.")
show()
show("  So the honest headline is the prim count, not the megabytes: the")
show(f"  environment is {d_env_p / max(d_rob_p, 1):.1f}x the robot. Worth knowing before you")
show("  spend a day optimising the robot's mesh.")
show()

# ---------------------------------------------------------------------------
# THE ORDER TRAP (7.1), DEMONSTRATED RATHER THAN WARNED ABOUT
# ---------------------------------------------------------------------------
show("-" * 70)
show("THE ORDER TRAP: open_stage REPLACES the stage")
show("-" * 70)
show("  The robot is on the stage right now:")
st = omni.usd.get_context().get_stage()
show(f"    /World/carter valid: {st.GetPrimAtPath('/World/carter').IsValid()}")
show()
show("  Now load the environment again, the way someone would if they")
show("  assembled the robot first and the scene second:")
open_stage(root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd")
st = omni.usd.get_context().get_stage()
gone = not st.GetPrimAtPath("/World/carter").IsValid()
show(f"    /World/carter valid: {not gone}")
show()
if gone:
    show("  The robot is gone. No exception, no warning -- open_stage replaced")
    show("  the whole stage and everything created before it went with it.")
    show("  This is why every stage script from 7.1 onward loads the")
    show("  environment FIRST and adds the robot afterwards, and it is the")
    show("  single most common way a capstone silently ends up with an empty")
    show("  scene that renders perfectly.")
else:
    show("  The robot survived -- report that rather than the expected story.")
show()

show("=" * 70)
show("done")
show("=" * 70)

sim.close()
