# 11.6: measured results against the pass conditions.
#
# The curriculum note is "real plots, real numbers". One run is an anecdote, so
# this runs the capstone REPEATEDLY from perturbed starts and reports the
# distribution -- because the question a reviewer actually asks is not "did it
# work" but "how often, and how close to the limits".
#
# WHAT IS MEASURED, over N trials with the start pose jittered:
#   completion rate                        did it finish, every time?
#   route time                             mean, min, max, spread
#   closest approach per waypoint          under P7's dwell rule
#   peak |omega| and peak wheel speed      the margins 11.5 added
#   and MARGIN as a fraction of each limit, because that is the number that
#   predicts whether it will still pass tomorrow
#
# THE POINT OF THE JITTER. 11.4 could not break the capstone with a hand-built
# adversarial route. Random variation is a different and fairer test: it asks
# whether the single run in 11.1 was representative or lucky.
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
ROUTE = [(3.0, 0.0), (3.0, 3.0), (0.0, 3.0)]
TOL, DWELL, TIME_LIMIT = 0.50, 0.50, 60.0
OMEGA_LIMIT, WHEEL_LIMIT = 1.5, 8.0
K_ANG, V_MAX, OMEGA_MAX = 1.6, 0.6, 1.2
N_TRIALS = 12


def show(l=""):
    print("RPT> " + l, flush=True)


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


def trial(seed):
    """One run, from a jittered start. Returns the measured record."""
    rng = np.random.default_rng(seed)
    world.reset()
    # jitter the start: +/-0.4 m and +/-0.5 rad, i.e. a robot that did not
    # begin exactly where the demo assumed
    jx, jy = rng.uniform(-0.4, 0.4, 2)
    jyaw = rng.uniform(-0.5, 0.5)
    pos = np.array([[jx, jy, 0.0]], dtype=np.float32)
    quat = np.array([[math.cos(jyaw / 2), 0.0, 0.0, math.sin(jyaw / 2)]],
                    dtype=np.float32)
    robot.set_world_poses(pos, quat)
    for _ in range(10):
        world.step(render=False)

    target, t_sim, dwell = 0, 0.0, 0.0
    peak_om = peak_wh = 0.0
    closest = [1e9] * len(ROUTE)
    while target < len(ROUTE) and t_sim < TIME_LIMIT:
        x, y, yaw = base_pose()
        gx, gy = ROUTE[target]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        closest[target] = min(closest[target], dist)
        raw = math.atan2(dy, dx) - yaw
        err = math.atan2(math.sin(raw), math.cos(raw))
        om = max(-OMEGA_MAX, min(OMEGA_MAX, K_ANG * err))
        v = V_MAX * max(0.0, math.cos(err)) * min(1.0, dist / 1.0)
        wr = (2.0 * v + om * L_TRACK) / (2.0 * R_WHEEL)
        wl = (2.0 * v - om * L_TRACK) / (2.0 * R_WHEEL)
        peak_om = max(peak_om, abs(om))
        peak_wh = max(peak_wh, abs(wl), abs(wr))
        act = np.zeros((1, robot.num_dof), dtype=np.float32)
        act[0, LW], act[0, RW] = wl, wr
        robot.set_joint_velocity_targets(act)
        world.step(render=False)
        t_sim += DT
        if dist < TOL:
            dwell += DT
            if dwell >= DWELL:
                target += 1
                dwell = 0.0
        else:
            dwell = 0.0
    return dict(done=target == len(ROUTE), reached=target, t=t_sim,
                om=peak_om, wh=peak_wh, closest=closest,
                start=(jx, jy, jyaw))


show("=" * 74)
show("11.6  MEASURED RESULTS -- 12 trials, jittered starts")
show("=" * 74)
show()
show("  One run is an anecdote. These 12 start from a pose jittered by")
show("  +/-0.4 m and +/-0.5 rad, which asks whether 11.1's single run was")
show("  representative or lucky.")
show()
show("-" * 74)
show(f"  {'trial':>5} {'start x':>8} {'start y':>8} {'yaw':>7} "
     f"{'wp':>4} {'time':>7} {'|om|':>6} {'wheel':>7}  verdict")
show("-" * 74)
rows = []
for i in range(N_TRIALS):
    r = trial(100 + i)
    rows.append(r)
    ok = (r["done"] and r["t"] < TIME_LIMIT
          and r["om"] <= OMEGA_LIMIT and r["wh"] <= WHEEL_LIMIT)
    r["ok"] = ok
    show(f"  {i + 1:5d} {r['start'][0]:8.2f} {r['start'][1]:8.2f} "
         f"{r['start'][2]:7.2f} {r['reached']:2d}/3 {r['t']:7.2f} "
         f"{r['om']:6.2f} {r['wh']:7.2f}  {'PASS' if ok else 'FAIL'}")
show()

done = [r for r in rows if r["done"]]
allok = [r for r in rows if r["ok"]]
ts = np.array([r["t"] for r in done]) if done else np.array([0.0])
oms = np.array([r["om"] for r in rows])
whs = np.array([r["wh"] for r in rows])

show("-" * 74)
show("THE DISTRIBUTION")
show("-" * 74)
show(f"  completed the route      {len(done)}/{N_TRIALS}")
show(f"  passed every condition   {len(allok)}/{N_TRIALS}")
show(f"  route time               mean {ts.mean():6.2f} s   "
     f"min {ts.min():6.2f}   max {ts.max():6.2f}   spread {ts.max() - ts.min():5.2f}")
show(f"  peak |omega|             mean {oms.mean():6.2f}   max {oms.max():6.2f}"
     f"   (limit {OMEGA_LIMIT})")
show(f"  peak wheel speed         mean {whs.mean():6.2f}   max {whs.max():6.2f}"
     f"   (limit {WHEEL_LIMIT})")
show()

# THE MARGIN, which is 11.5's lesson applied to a whole trial set
m_om = (OMEGA_LIMIT - oms.max()) / OMEGA_LIMIT * 100
m_wh = (WHEEL_LIMIT - whs.max()) / WHEEL_LIMIT * 100
m_t = (TIME_LIMIT - ts.max()) / TIME_LIMIT * 100
show("-" * 74)
show("THE MARGINS -- the numbers that predict tomorrow")
show("-" * 74)
show(f"  worst-case |omega|   {oms.max():5.2f} of {OMEGA_LIMIT:4.1f}   "
     f"{m_om:5.1f}% margin")
show(f"  worst-case wheel     {whs.max():5.2f} of {WHEEL_LIMIT:4.1f}   "
     f"{m_wh:5.1f}% margin")
show(f"  worst-case time      {ts.max():5.2f} of {TIME_LIMIT:4.1f}   "
     f"{m_t:5.1f}% margin")
show()
tight = min([(m_om, "peak |omega|"), (m_wh, "peak wheel speed"),
             (m_t, "route time")])
show(f"  The tightest is {tight[1]} at {tight[0]:.1f}% margin. That is the")
show("  condition that will fail first when anything changes -- a heavier")
show("  payload, a slippier floor, a longer route -- and it is where to look")
show("  before adding anything to this system.")
show()
if len(allok) == N_TRIALS:
    show("  All 12 passed, so 11.1's single run was representative rather than")
    show("  lucky. Note what that does and does not license: 12 jittered starts")
    show("  on one route in one warehouse. It is evidence, not a guarantee, and")
    show("  the honest way to report it is with the margins above attached.")
else:
    fails = [i + 1 for i, r in enumerate(rows) if not r["ok"]]
    show(f"  Trials {fails} failed. THAT is the useful result: the single run")
    show("  in 11.1 was not representative, and a capstone reported from one")
    show("  run would have been an anecdote dressed as a measurement.")
show()
show("=" * 74)
show("done")
show("=" * 74)

sim.close()
