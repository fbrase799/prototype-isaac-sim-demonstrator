# 11.4: running it, and the first honest failure.
#
# 11.3 ablated the loop and NOTHING broke. Every guard could be deleted and the
# capstone still passed. That is not evidence the guards are useless -- it is
# evidence the TEST is too easy, and this lecture fixes the test rather than
# congratulating the robot.
#
# THE HYPOTHESIS, written before running: each guard exists for a specific
# situation the current route never creates. So construct the situation.
#
#   the angle wrap        matters when the required turn crosses +/-pi, i.e.
#                         when the waypoint is BEHIND the robot
#   the omega clamp       matters when the heading error is large, because the
#                         proportional term is then huge
#   the cos(err) term     matters when the robot must turn far before driving,
#                         or it drives away at full speed while turning
#   the distance taper    matters when waypoints are CLOSE together, because
#                         there is no room to decelerate
#
# So: a route that starts the robot facing away, puts waypoints close together,
# and requires reversals. Same loop, same pass conditions, harder route.
from isaacsim import SimulationApp

sim = SimulationApp({"headless": True})

import math

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage, open_stage
from isaacsim.core.prims import Articulation
from isaacsim.storage.native import get_assets_root_path

CARTER = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
          "/Assets/Isaac/4.5/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd")
R_WHEEL, L_TRACK = 0.1400, 0.4132
TOL, TIME_LIMIT = 0.50, 60.0
K_ANG, V_MAX, OMEGA_MAX = 1.6, 0.6, 1.2

EASY = [(3.0, 0.0), (3.0, 3.0), (0.0, 3.0)]
# the hard route: behind the robot, then a tight zig-zag with 0.8 m legs
HARD = [(-2.0, 0.0), (-2.8, 0.8), (-2.0, 1.6), (-2.8, 2.4)]


def show(l=""):
    print("BRK> " + l, flush=True)


root = get_assets_root_path()
open_stage(root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd")
world = World(stage_units_in_meters=1.0)
add_reference_to_stage(usd_path=CARTER, prim_path="/World/carter")
robot = Articulation(prim_paths_expr="/World/carter", name="carter")
world.scene.add(robot)
world.reset()
names = list(robot.dof_names)
LW, RW = names.index("joint_wheel_left"), names.index("joint_wheel_right")
DT = world.get_physics_dt()


def base_pose():
    p = robot.get_world_poses()
    pos = np.array(p[0][0], dtype=float)
    w, x, y, z = np.array(p[1][0], dtype=float)
    return pos[0], pos[1], math.atan2(2.0 * (w * z + x * y),
                                      1.0 - 2.0 * (y * y + z * z))


def run(route, tag, cos_term=True, dist_taper=True, wrap=True, clamp=True,
        quiet=False):
    world.reset()
    for _ in range(10):
        world.step(render=False)
    target, t_sim = 0, 0.0
    peak_omega = 0.0
    while target < len(route) and t_sim < TIME_LIMIT:
        x, y, yaw = base_pose()
        gx, gy = route[target]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        raw = math.atan2(dy, dx) - yaw
        err = math.atan2(math.sin(raw), math.cos(raw)) if wrap else raw
        om = K_ANG * err
        if clamp:
            om = max(-OMEGA_MAX, min(OMEGA_MAX, om))
        peak_omega = max(peak_omega, abs(om))
        v = V_MAX
        if cos_term:
            v *= max(0.0, math.cos(err))
        if dist_taper:
            v *= min(1.0, dist / 1.0)
        wr = (2.0 * v + om * L_TRACK) / (2.0 * R_WHEEL)
        wl = (2.0 * v - om * L_TRACK) / (2.0 * R_WHEEL)
        act = np.zeros((1, robot.num_dof), dtype=np.float32)
        act[0, LW], act[0, RW] = wl, wr
        robot.set_joint_velocity_targets(act)
        world.step(render=False)
        t_sim += DT
        if dist < TOL:
            target += 1
    ok = target == len(route)
    if not quiet:
        show(f"  {tag:36s} {target}/{len(route)}   {t_sim:6.2f} s   "
             f"peak |omega| {peak_omega:6.2f}   {'PASS' if ok else 'FAIL'}")
    return target, t_sim, ok, peak_omega


show("=" * 70)
show("11.4  BREAKING IT -- the test was too easy, so make it hard")
show("=" * 70)
show()
show("  11.3 deleted every guard in the loop and the capstone still passed.")
show("  So the guards are not useless -- the ROUTE never exercised them.")
show()
show("  HYPOTHESIS, written before running: each guard has a situation.")
show("    angle wrap   -> a waypoint BEHIND the robot (turn crosses +/-pi)")
show("    omega clamp  -> a large heading error (the P term goes huge)")
show("    cos(err)     -> a big turn before driving")
show("    dist taper   -> waypoints CLOSE together, no room to decelerate")
show()
show("  So build that route: start facing away, 0.8 m legs, reversals.")
show()

show("-" * 70)
show("THE EASY ROUTE (11.1's), FOR REFERENCE")
show("-" * 70)
run(EASY, "full loop, easy route")
show()

show("-" * 70)
show("THE HARD ROUTE, SAME LOOP")
show("-" * 70)
full = run(HARD, "full loop, hard route")
show()

show("-" * 70)
show("THE HARD ROUTE, ONE GUARD REMOVED AT A TIME")
show("-" * 70)
a = run(HARD, "A  without cos(err) speed term", cos_term=False)
b = run(HARD, "B  without the distance taper", dist_taper=False)
c = run(HARD, "C  without the angle wrap", wrap=False)
d = run(HARD, "D  without the omega clamp", clamp=False)
show()

show("-" * 70)
show("WHAT CHANGED")
show("-" * 70)
res = [("A  cos(err) speed term", a), ("B  distance taper", b),
       ("C  angle wrap", c), ("D  omega clamp", d)]
broke = [(n, r) for n, r in res if not r[2]]
if not full[2]:
    show(f"  THE FULL LOOP ITSELF FAILS the hard route: {full[0]}/{len(HARD)}")
    show(f"  waypoints in {full[1]:.2f} s. That is the honest failure this")
    show("  lecture was looking for, and 11.5 fixes it on camera.")
    show()
elif broke:
    show("  The full loop still passes, but the guards now MATTER:")
    for n, r in broke:
        show(f"    removing {n:28s} -> {r[0]}/{len(HARD)} waypoints, FAIL")
    show()
    show("  That is the point of 11.3's null result. Those lines were not")
    show("  redundant; the easy route simply never asked anything of them.")
    show("  A test that cannot fail cannot tell you which parts are load-")
    show("  bearing, and 'it passed' is not evidence that any line earned")
    show("  its place.")
else:
    show("  EVEN THE HARD ROUTE DID NOT BREAK ANYTHING, and I am reporting")
    show("  that rather than claiming a failure I did not produce. My")
    show("  hypothesis was that a reversal and 0.8 m legs would make the")
    show("  guards load-bearing. It did not.")
    show()
    # BUT THE PEAK TURN RATE IS THE EVIDENCE, and it is worth more than a
    # pass/fail flag: the clamp is visibly doing work even when the run
    # survives without it.
    show("  BUT LOOK AT THE PEAK TURN RATE, which pass/fail throws away:")
    for n, r in res:
        show(f"    {n:30s} peak |omega| {r[3]:5.2f} rad/s")
    show(f"    full loop                      peak |omega| {full[3]:5.2f} rad/s")
    show()
    ratio = d[3] / max(full[3], 1e-9)
    show(f"  Removing the clamp lets the turn rate reach {d[3]:.2f} rad/s against")
    show(f"  {full[3]:.2f} with it -- {ratio:.1f}x. The robot still reaches every")
    show("  waypoint, so the pass condition never notices, and on a real")
    show("  machine that is the difference between a turn and a wheel skid.")
    show()
    show("  THAT IS THE HONEST LESSON OF THIS LECTURE. A binary pass condition")
    show("  cannot see a guard doing its job; it only sees the run survive.")
    show("  If a line matters, measure the quantity it BOUNDS, not the outcome")
    show("  it protects. Getting a capstone to fail on demand also turns out")
    show("  to be harder than it looks -- the next honest step would need")
    show("  obstacles, and a planner is explicitly out of scope (9.9).")
show()
show("=" * 70)
show("done")
show("=" * 70)

sim.close()
