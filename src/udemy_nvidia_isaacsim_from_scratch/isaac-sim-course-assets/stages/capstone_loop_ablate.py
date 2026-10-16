# 11.3: the autonomy loop, line by line -- and what each line is worth.
#
# The loop is four lines of logic. Describing them takes thirty seconds and
# teaches nothing, because a learner cannot tell which lines are load-bearing
# and which are decoration. So this script ABLATES it: removes one line at a
# time and re-runs the capstone, reporting what breaks.
#
# THE FOUR LINES, as written in capstone.py:
#   1. bearing = atan2(dy, dx)              where is the waypoint
#   2. err     = wrap(bearing - yaw)        how far off are we pointing
#   3. omega   = clamp(K_ANG * err)         turn toward it, proportionally
#   4. v       = V_MAX * cos(err) * min(1, dist/1.0)
#                                           slow down when badly aimed, and
#                                           when close
#
# ABLATIONS, each a single deliberate deletion:
#   A  no cos(err) term      drive at full speed regardless of heading
#   B  no dist taper         drive at full speed right up to the waypoint
#   C  no angle wrap         subtract the angles naively
#   D  no omega clamp        let the turn rate go wherever it wants
#
# Each is re-run against the SAME pass conditions from 11.1, so "what does this
# line do" is answered in waypoints reached and seconds taken rather than in
# adjectives.
from isaacsim import SimulationApp

sim = SimulationApp({"headless": True})

import math
import time

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage, open_stage
from isaacsim.core.prims import Articulation
from isaacsim.storage.native import get_assets_root_path

CARTER = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
          "/Assets/Isaac/4.5/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd")
R_WHEEL, L_TRACK = 0.1400, 0.4132
WAYPOINTS = [(3.0, 0.0), (3.0, 3.0), (0.0, 3.0)]
TOL, TIME_LIMIT = 0.50, 60.0
K_ANG, V_MAX, OMEGA_MAX = 1.6, 0.6, 1.2


def show(l=""):
    print("ABL> " + l, flush=True)


root = get_assets_root_path()
open_stage(root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd")
world = World(stage_units_in_meters=1.0)
add_reference_to_stage(usd_path=CARTER, prim_path="/World/carter")
robot = Articulation(prim_paths_expr="/World/carter", name="carter")
world.scene.add(robot)
world.reset()
names = list(robot.dof_names)
LW = names.index("joint_wheel_left")
RW = names.index("joint_wheel_right")
DT = world.get_physics_dt()


def base_pose():
    p = robot.get_world_poses()
    pos = np.array(p[0][0], dtype=float)
    w, x, y, z = np.array(p[1][0], dtype=float)
    return pos[0], pos[1], math.atan2(2.0 * (w * z + x * y),
                                      1.0 - 2.0 * (y * y + z * z))


def run(tag, cos_term=True, dist_taper=True, wrap=True, clamp=True):
    world.reset()
    for _ in range(10):
        world.step(render=False)
    target, t_sim = 0, 0.0
    while target < len(WAYPOINTS) and t_sim < TIME_LIMIT:
        x, y, yaw = base_pose()
        gx, gy = WAYPOINTS[target]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        raw = bearing - yaw
        err = math.atan2(math.sin(raw), math.cos(raw)) if wrap else raw
        om = K_ANG * err
        if clamp:
            om = max(-OMEGA_MAX, min(OMEGA_MAX, om))
        v = V_MAX
        if cos_term:
            v *= max(0.0, math.cos(err))
        if dist_taper:
            v *= min(1.0, dist / 1.0)
        wr = (2.0 * v + om * L_TRACK) / (2.0 * R_WHEEL)
        wl = (2.0 * v - om * L_TRACK) / (2.0 * R_WHEEL)
        act = np.zeros((1, robot.num_dof), dtype=np.float32)
        act[0, LW] = wl
        act[0, RW] = wr
        robot.set_joint_velocity_targets(act)
        world.step(render=False)
        t_sim += DT
        if dist < TOL:
            target += 1
    ok = target == len(WAYPOINTS)
    show(f"  {tag:34s} {target}/3 waypoints   {t_sim:6.2f} s   "
         f"{'PASS' if ok else 'FAIL'}")
    return target, t_sim, ok


show("=" * 70)
show("11.3  THE LOOP, ABLATED -- what is each line actually worth?")
show("=" * 70)
show()
show("-" * 70)
show("REMOVE ONE LINE AT A TIME, SAME PASS CONDITIONS")
show("-" * 70)
base = run("the loop as written")
a = run("A  without cos(err) speed term", cos_term=False)
b = run("B  without the distance taper", dist_taper=False)
c = run("C  without the angle wrap", wrap=False)
d = run("D  without the omega clamp", clamp=False)
show()

show("-" * 70)
show("WHAT THAT SAYS")
show("-" * 70)
res = [("A  cos(err) speed term", a), ("B  distance taper", b),
       ("C  angle wrap", c), ("D  omega clamp", d)]
broke = [n for n, r in res if not r[2]]
slower = [(n, r[1] - base[1]) for n, r in res if r[2] and r[1] > base[1] + 0.5]
if broke:
    show("  LOAD-BEARING -- removing these FAILS the capstone:")
    for n in broke:
        show(f"    {n}")
else:
    show("  Nothing broke the run outright, which is itself a finding: this")
    show("  loop is more forgiving than it looks on a clear floor.")
show()
if slower:
    show("  COSTLY BUT SURVIVABLE -- still passes, takes longer:")
    for n, d_ in slower:
        show(f"    {n:34s} +{d_:.2f} s")
    show()
show("  Any line whose removal changes nothing is a line you do not")
show("  understand yet -- either it is genuinely redundant here, or the test")
show("  does not exercise it. On a clear warehouse floor with three")
show("  well-spaced waypoints, several of these guards never get to matter.")
show("  That is a property of the TEST, not evidence the guards are useless,")
show("  and 11.4 goes looking for the case that makes each one earn its keep.")
show()
show("=" * 70)
show("done")
show("=" * 70)

sim.close()
