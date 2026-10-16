# 11.8: training a first policy, small.
#
# 11.7 measured what RL costs here: ~735 env-steps/s, ~90 MiB per environment,
# and 100M steps would take 37.8 hours. So the task has to fit that budget, and
# the honest way to choose one is to work backwards from the number rather than
# to pick something impressive and hope.
#
# THE TASK, scoped deliberately. The capstone's controller turns toward a
# waypoint using a gain I chose by hand (K_ANG = 1.6) and a speed profile I
# wrote. This trains a policy to do the SAME job -- map (distance, heading
# error) to (v, omega) -- on the same robot, and then compares it against the
# hand-written controller on the same route with the same pass conditions.
#
# WHY THIS TASK AND NOT LOCOMOTION. Two reasons, both honest:
#   1. it fits. A 2-in, 2-out policy on a few hundred thousand steps is hours,
#      not days, at the rate 11.7 measured.
#   2. it has a BASELINE. We already know what good looks like, because the
#      hand-written controller passes with a 20% margin. A first RL result with
#      nothing to compare against tells you almost nothing.
#
# WHAT IS MEASURED, and reported whatever it says:
#   - the learning curve, over real environment steps
#   - the learned policy against the hand-written one, on the capstone route,
#     against 11.5's conditions including peak |omega| and peak wheel speed
#   - the wall-clock cost, so the "is this worth it" question has a number
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
ROUTE = [(3.0, 0.0), (3.0, 3.0), (0.0, 3.0)]
TOL, DWELL, TIME_LIMIT = 0.50, 0.50, 60.0
OMEGA_LIMIT, WHEEL_LIMIT = 1.5, 8.0
K_ANG, V_MAX, OMEGA_MAX = 1.6, 0.6, 1.2


def show(l=""):
    print("RL8> " + l, flush=True)


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


def apply(v, om):
    om = max(-OMEGA_MAX, min(OMEGA_MAX, om))
    wr = (2.0 * v + om * L_TRACK) / (2.0 * R_WHEEL)
    wl = (2.0 * v - om * L_TRACK) / (2.0 * R_WHEEL)
    act = np.zeros((1, robot.num_dof), dtype=np.float32)
    act[0, LW], act[0, RW] = wl, wr
    robot.set_joint_velocity_targets(act)
    return abs(om), max(abs(wl), abs(wr))


show("=" * 74)
show("11.8  A FIRST POLICY, SCOPED TO FIT")
show("=" * 74)
show()
show("  11.7 measured ~735 env-steps/s and ~90 MiB per environment here, so")
show("  the task is chosen to FIT that budget rather than to look impressive.")
show()
show("  The task: map (distance, heading error) -> (v, omega) on the capstone")
show("  robot -- the same job the hand-written controller does. It fits, and")
show("  crucially it has a BASELINE, because we already know what good looks")
show("  like: the hand-written loop passes with a 20% margin.")
show()

import torch
import torch.nn as nn

torch.manual_seed(0)
policy = nn.Sequential(nn.Linear(2, 16), nn.Tanh(),
                       nn.Linear(16, 16), nn.Tanh(),
                       nn.Linear(16, 2), nn.Tanh())
nparam = sum(p.numel() for p in policy.parameters())
show(f"  the policy: {nparam} parameters, 2 in, 2 out")
show()

# ---------------------------------------------------------------------------
# BEHAVIOUR CLONING, which is the honest choice at this budget.
# A from-scratch policy-gradient run needs far more steps than 11.7's numbers
# allow. Cloning the controller we already have needs thousands, not millions,
# and it answers a real question: can a network represent this controller at
# all, and what does it cost to run?
# ---------------------------------------------------------------------------
show("-" * 74)
show("1. TRAINING")
show("-" * 74)
show("  Method: behaviour cloning from the hand-written controller.")
show("  A from-scratch policy gradient at ~735 steps/s would need days for")
show("  this to converge (11.7's arithmetic). Cloning needs thousands of")
show("  samples, and it answers the question that actually matters first:")
show("  CAN a small network represent this controller, and what does it cost?")
show()

rng = np.random.default_rng(0)
N = 20000
d = rng.uniform(0.0, 4.0, N)
e = rng.uniform(-math.pi, math.pi, N)
v_t = V_MAX * np.maximum(0.0, np.cos(e)) * np.minimum(1.0, d / 1.0)
om_t = np.clip(K_ANG * e, -OMEGA_MAX, OMEGA_MAX)
X = torch.tensor(np.stack([d / 4.0, e / math.pi], axis=1), dtype=torch.float32)
Y = torch.tensor(np.stack([v_t / V_MAX, om_t / OMEGA_MAX], axis=1),
                 dtype=torch.float32)

opt = torch.optim.Adam(policy.parameters(), lr=3e-3)
lf = nn.MSELoss()
t0 = time.perf_counter()
curve = []
for ep in range(400):
    perm = torch.randperm(len(X))
    for i in range(0, len(perm), 256):
        idx = perm[i:i + 256]
        opt.zero_grad()
        loss = lf(policy(X[idx]), Y[idx])
        loss.backward()
        opt.step()
    if ep % 100 == 99 or ep == 0:
        with torch.no_grad():
            l = float(lf(policy(X), Y))
        curve.append((ep + 1, l))
        show(f"    epoch {ep + 1:4d}   loss {l:.6f}")
train_s = time.perf_counter() - t0
show(f"  trained in {train_s:.1f} s on CPU")
show()


# ---------------------------------------------------------------------------
# 2. RUN BOTH ON THE CAPSTONE ROUTE
# ---------------------------------------------------------------------------
def run(use_policy):
    world.reset()
    for _ in range(10):
        world.step(render=False)
    target, t_sim, dwell = 0, 0.0, 0.0
    pk_om = pk_wh = 0.0
    while target < len(ROUTE) and t_sim < TIME_LIMIT:
        x, y, yaw = base_pose()
        gx, gy = ROUTE[target]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        raw = math.atan2(dy, dx) - yaw
        err = math.atan2(math.sin(raw), math.cos(raw))
        if use_policy:
            with torch.no_grad():
                out = policy(torch.tensor([[min(dist, 4.0) / 4.0,
                                            err / math.pi]],
                                          dtype=torch.float32))[0].numpy()
            v, om = float(out[0]) * V_MAX, float(out[1]) * OMEGA_MAX
            v = max(0.0, v)
        else:
            om = K_ANG * err
            v = V_MAX * max(0.0, math.cos(err)) * min(1.0, dist / 1.0)
        a, b = apply(v, om)
        pk_om, pk_wh = max(pk_om, a), max(pk_wh, b)
        world.step(render=False)
        t_sim += DT
        if dist < TOL:
            dwell += DT
            if dwell >= DWELL:
                target += 1
                dwell = 0.0
        else:
            dwell = 0.0
    ok = (target == len(ROUTE) and t_sim < TIME_LIMIT
          and pk_om <= OMEGA_LIMIT and pk_wh <= WHEEL_LIMIT)
    return target, t_sim, pk_om, pk_wh, ok


show("-" * 74)
show("2. BOTH CONTROLLERS, SAME ROUTE, SAME CONDITIONS")
show("-" * 74)
h = run(False)
p = run(True)
for tag, r in (("hand-written", h), ("learned policy", p)):
    show(f"  {tag:16s} {r[0]}/3  {r[1]:6.2f} s  |om| {r[2]:5.2f}  "
         f"wheel {r[3]:5.2f}  {'PASS' if r[4] else 'FAIL'}")
show()

show("-" * 74)
show("WHAT THAT SAYS")
show("-" * 74)
if p[4] and h[4]:
    dt_ = p[1] - h[1]
    show(f"  Both pass. The learned policy takes {dt_:+.2f} s against the")
    show("  hand-written one, and respects the same limits.")
    show()
    show("  BE CLEAR ABOUT WHAT WAS AND WAS NOT SHOWN. This policy was CLONED")
    show(f"  from the controller, so the result is that a {nparam}-parameter")
    show("  network can represent a controller I already had. It is not")
    show("  evidence that")
    show("  RL discovers controllers, because no reward was optimised and no")
    show("  exploration happened.")
    show()
    show("  That distinction matters more than the result. Behaviour cloning")
    show("  answers 'can this be represented and run in the loop'. Policy")
    show("  gradient answers 'can this be discovered', and 11.7's arithmetic")
    show("  says discovering it here would take days rather than seconds.")
elif not p[4]:
    show(f"  The learned policy FAILED: {p[0]}/3 waypoints, |om| {p[2]:.2f},")
    show(f"  wheel {p[3]:.2f}. Reported as-is.")
    show()
    show("  A cloned policy that fails the conditions its teacher passes is a")
    show("  real and common outcome: small errors compound around the loop,")
    show("  because the network sees states its training set never covered.")
    show("  That is distribution shift, and it is the single biggest reason")
    show("  imitation learning is harder than it looks.")
else:
    show("  The hand-written controller failed, which means something changed")
    show("  in the setup -- investigate that before reading anything into the")
    show("  policy's result.")
show()
show(f"  Cost: {train_s:.1f} s of CPU training and {N} samples generated from")
show("  a controller that already existed. Against 11.7's 37.8 hours for a")
show("  100M-step run, that is the whole argument for starting small.")
show()
show("=" * 74)
show("done")
show("=" * 74)

sim.close()
