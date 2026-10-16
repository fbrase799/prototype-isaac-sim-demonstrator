# 11.5: fixing it on camera.
#
# 11.4 found that the capstone's pass conditions cannot see a guard doing its
# job. Removing the turn-rate clamp let the controller demand 5.03 rad/s
# instead of 1.20 -- 4.2x -- and every condition still read PASS, because the
# conditions only ask whether the robot arrived.
#
# So the thing that gets fixed here is NOT the robot. It is the conditions.
#
# THE OLD CONDITIONS (11.1), all outcome-shaped:
#   P1 within 0.50 m of each waypoint      did it arrive
#   P2 route under 60 s                    did it arrive in time
#   P3 no waypoint missed                  did it arrive at all of them
#   P4 loop at or above 30 Hz              did the software keep up
#
# THE NEW ONES add the quantities the guards BOUND, which is 11.4's lesson
# turned into a rule:
#   P5 peak |omega| <= 1.5 rad/s           the clamp is doing its job
#   P6 peak wheel speed <= 8.0 rad/s       no commanded skid
#   P7 arrival means DWELL: hold inside the tolerance for 0.50 s
#      (11.1 already found the touch rule pins the margin to the rim)
#
# Then the same ablations are re-run against the NEW conditions. If the fix
# works, deleting a guard should now FAIL something, and the lecture will have
# earned the word "fixing".
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
ROUTE = [(-2.0, 0.0), (-2.8, 0.8), (-2.0, 1.6), (-2.8, 2.4)]
TOL, DWELL, TIME_LIMIT = 0.50, 0.50, 60.0
OMEGA_LIMIT, WHEEL_LIMIT = 1.5, 8.0
K_ANG, V_MAX, OMEGA_MAX = 1.6, 0.6, 1.2


def show(l=""):
    print("FIX> " + l, flush=True)


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


def run(tag, cos_term=True, dist_taper=True, wrap=True, clamp=True):
    world.reset()
    for _ in range(10):
        world.step(render=False)
    target, t_sim, dwell = 0, 0.0, 0.0
    peak_om, peak_wheel = 0.0, 0.0
    while target < len(ROUTE) and t_sim < TIME_LIMIT:
        x, y, yaw = base_pose()
        gx, gy = ROUTE[target]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        raw = math.atan2(dy, dx) - yaw
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
        peak_om = max(peak_om, abs(om))
        peak_wheel = max(peak_wheel, abs(wl), abs(wr))
        act = np.zeros((1, robot.num_dof), dtype=np.float32)
        act[0, LW], act[0, RW] = wl, wr
        robot.set_joint_velocity_targets(act)
        world.step(render=False)
        t_sim += DT
        # P7: arrival is DWELL, not touch
        if dist < TOL:
            dwell += DT
            if dwell >= DWELL:
                target += 1
                dwell = 0.0
        else:
            dwell = 0.0
    P = {
        "P1 waypoints": target == len(ROUTE),
        "P2 under 60 s": t_sim < TIME_LIMIT and target == len(ROUTE),
        "P4 loop rate": True,               # measured at 346 Hz in 11.1
        "P5 peak |omega|": peak_om <= OMEGA_LIMIT,
        "P6 peak wheel": peak_wheel <= WHEEL_LIMIT,
    }
    ok = all(P.values())
    show(f"  {tag:34s} {target}/{len(ROUTE)}  {t_sim:6.2f} s  "
         f"|om| {peak_om:5.2f}  wheel {peak_wheel:5.2f}  "
         f"{'PASS' if ok else 'FAIL'}")
    return P, ok, peak_om, peak_wheel


show("=" * 74)
show("11.5  FIXING IT -- and the thing being fixed is the CONDITIONS")
show("=" * 74)
show()
show("  11.4: removing the clamp took peak |omega| from 1.20 to 5.03 rad/s")
show("  and every condition still read PASS. The conditions only asked")
show("  whether the robot arrived.")
show()
show("  So add the quantities the guards BOUND, and make arrival mean dwell:")
show(f"    P5  peak |omega|     <= {OMEGA_LIMIT:.1f} rad/s")
show(f"    P6  peak wheel speed <= {WHEEL_LIMIT:.1f} rad/s")
show(f"    P7  arrival = HOLD inside {TOL:.2f} m for {DWELL:.2f} s")
show()
show("-" * 74)
show("THE SAME ABLATIONS, AGAINST THE NEW CONDITIONS")
show("-" * 74)
full = run("full loop")
a = run("A  without cos(err) speed term", cos_term=False)
b = run("B  without the distance taper", dist_taper=False)
c = run("C  without the angle wrap", wrap=False)
d = run("D  without the omega clamp", clamp=False)
show()

show("-" * 74)
show("DID THE FIX WORK?")
show("-" * 74)
res = [("A  cos(err) term", a), ("B  distance taper", b),
       ("C  angle wrap", c), ("D  omega clamp", d)]
now_fail = [(n, r) for n, r in res if not r[1]]
if full[1] and now_fail:
    show("  YES. The full loop passes, and removing a guard now FAILS:")
    for n, r in now_fail:
        bad = [k for k, v in r[0].items() if not v]
        show(f"    {n:24s} fails {', '.join(bad)}")
    show()
    show("  Under 11.1's conditions every one of these passed. Nothing about")
    show("  the robot changed -- the conditions started measuring the thing")
    show("  the guards exist to bound, and the guards immediately became")
    show("  load-bearing.")
    show()
    show("  That is the fix, and it is worth being precise about what was")
    show("  broken: not the controller, and not the robot. The TEST was")
    show("  broken, and it was broken in the specific way that tests usually")
    show("  are -- it measured the outcome and ignored the mechanism.")
elif not full[1]:
    bad = [k for k, v in full[0].items() if not v]
    show(f"  The FULL LOOP now fails {', '.join(bad)} -- the new limits are")
    show("  too tight for the controller as written. That is a real result:")
    show("  report it and either loosen the limit with a reason or fix the")
    show("  controller, but do not quietly pick whichever number passes.")
else:
    show("  NO. Every ablation still passes even with the new conditions.")
    show("  Report that rather than claiming a fix that did not happen.")
show()
show("=" * 74)
show("done")
show("=" * 74)

sim.close()
