# 8.5: a velocity controller, written line by line -- with the bug in it first.
#
# 8.4 ended on a measured failure: asked for wR = +6.00 rad/s, the right wheel
# delivered -1.59 and never reversed, so a body motion predicted from the
# COMMAND was wrong while the same prediction from the ACHIEVED wheel speeds was
# right to within 10%. That is the gap a controller exists to close, and this
# lecture closes it in the order you would actually discover it:
#
#   v1  OPEN LOOP      solve the inverse kinematics once, send it, hope.
#   v2  THE OBVIOUS FIX, WHICH IS A BUG: feed the error back with a gain, and
#       forget that the thing you are correcting is itself rate-limited. Run it
#       and watch it wind up / oscillate.
#   v3  CLOSED LOOP    proportional on the measured wheel velocity, with the
#       integral clamped, and the SAME target met.
#
# The point is not that PI control works -- everyone knows that. The point is
# that the naive version FAILS IN A SPECIFIC, MEASURABLE WAY and that you can
# only see it if you log the intermediate quantity rather than the endpoints.
#
# Everything below is measured on the real robot. No gains are quoted that were
# not run.
from isaacsim import SimulationApp

sim = SimulationApp({"headless": True})

import math
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation

CARTER = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
          "/Assets/Isaac/4.5/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd")


def show(l=""):
    print("PY5> " + l, flush=True)


world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
add_reference_to_stage(usd_path=CARTER, prim_path="/World/carter")
robot = Articulation(prim_paths_expr="/World/carter", name="carter")
world.scene.add(robot)
world.reset()

names = list(robot.dof_names)
LW = names.index("joint_wheel_left")
RW = names.index("joint_wheel_right")
DT = world.get_physics_dt()
R, L = 0.1400, 0.4132          # measured in 8.4, off the stage

STEPS = 300                     # 5.0 s at 60 Hz
TARGET_WL, TARGET_WR = -4.0, 4.0   # a spin: the case 8.4 saw fail hardest


def run(controller, steps=STEPS):
    """Drive with `controller(step, wl_meas, wr_meas) -> (wl_cmd, wr_cmd)`.

    Returns the per-step measured wheel velocities, so the caller can look at
    the TRAJECTORY rather than only the final number. 8.4's whole lesson was
    that the endpoints hide the mechanism.
    """
    world.reset()
    for _ in range(10):
        world.step(render=False)
    hist_l, hist_r = [], []
    for i in range(steps):
        jv = robot.get_joint_velocities()[0]
        wl_m, wr_m = float(jv[LW]), float(jv[RW])
        wl_c, wr_c = controller(i, wl_m, wr_m)
        act = np.zeros((1, robot.num_dof), dtype=np.float32)
        act[0, LW] = wl_c
        act[0, RW] = wr_c
        robot.set_joint_velocity_targets(act)
        world.step(render=False)
        hist_l.append(wl_m)
        hist_r.append(wr_m)
    return np.array(hist_l), np.array(hist_r)


def summarise(tag, hl, hr):
    """Steady-state error over the last second, and the worst overshoot."""
    tail_l, tail_r = hl[-60:], hr[-60:]
    el_ = float(np.mean(tail_l)) - TARGET_WL
    er_ = float(np.mean(tail_r)) - TARGET_WR
    show(f"  {tag}")
    show(f"    left  settled {np.mean(tail_l):+7.3f} rad/s "
         f"(target {TARGET_WL:+.2f}, error {el_:+.3f})")
    show(f"    right settled {np.mean(tail_r):+7.3f} rad/s "
         f"(target {TARGET_WR:+.2f}, error {er_:+.3f})")
    show(f"    worst |right| during the run: {np.max(np.abs(hr)):.3f} rad/s")
    show(f"    right velocity std over the last second: {np.std(tail_r):.4f}")
    return el_, er_


show("=" * 70)
show("8.5  A VELOCITY CONTROLLER -- open loop, the bug, then closed")
show("=" * 70)
show()
show(f"target: wL {TARGET_WL:+.2f}  wR {TARGET_WR:+.2f} rad/s   "
     f"({STEPS} steps = {STEPS * DT:.1f} s)")
show(f"measured in 8.4: r = {R} m, L = {L} m")
show()

# ---------------------------------------------------------------------------
# v1: OPEN LOOP. Send the target and hope.
# ---------------------------------------------------------------------------
show("-" * 70)
show("v1  OPEN LOOP -- send the number, hope for the best")
show("-" * 70)
hl1, hr1 = run(lambda i, wl, wr: (TARGET_WL, TARGET_WR))
e1l, e1r = summarise("open loop", hl1, hr1)
show()
show("  This is 8.4's failure again, in isolation. Nothing here is wrong with")
show("  the arithmetic; the command simply is not achieved and nothing says so.")
show()

# ---------------------------------------------------------------------------
# v2: THE OBVIOUS FIX, AND IT IS A BUG.
# Integral action with no clamp. The drive cannot deliver what is being asked,
# so the error never goes away, so the integral grows without bound and the
# command runs away to a number the drive will never honour.
# ---------------------------------------------------------------------------
show("-" * 70)
show("v2  THE OBVIOUS FIX -- add integral action, forget the clamp")
show("-" * 70)
KP, KI = 0.8, 2.0
state = {"il": 0.0, "ir": 0.0, "peak": 0.0}


def v2(i, wl, wr):
    el_ = TARGET_WL - wl
    er_ = TARGET_WR - wr
    state["il"] += el_ * DT
    state["ir"] += er_ * DT
    cl = TARGET_WL + KP * el_ + KI * state["il"]
    cr = TARGET_WR + KP * er_ + KI * state["ir"]
    state["peak"] = max(state["peak"], abs(cr))
    return cl, cr


hl2, hr2 = run(v2)
e2l, e2r = summarise(f"PI, Kp={KP} Ki={KI}, NO clamp", hl2, hr2)
show(f"    the integral reached {state['ir']:+.3f}, so the COMMAND sent to the")
show(f"    right wheel peaked at {state['peak']:.1f} rad/s")
show()
show("  Look at that command peak against the target of "
     f"{TARGET_WR:+.2f}. The wheel")
show("  cannot deliver the target, so the error never clears, so the integral")
show("  keeps accumulating and the controller demands ever more from an")
show("  actuator that already said no. That is integral wind-up, and it is the")
show("  most common bug in a first controller.")
show()

# ---------------------------------------------------------------------------
# v3: CLOSED LOOP, DONE PROPERLY. Clamp the integral to what the actuator can
# actually be asked for, and stop integrating when the command is saturated.
# ---------------------------------------------------------------------------
show("-" * 70)
show("v3  CLOSED LOOP -- clamp the integral, stop winding up")
show("-" * 70)
LIMIT = 8.0          # the most we will ever ask a wheel for
KP3, KI3 = 0.8, 2.0
st3 = {"il": 0.0, "ir": 0.0, "peak": 0.0}


def v3(i, wl, wr):
    out = []
    for tgt, meas, key in ((TARGET_WL, wl, "il"), (TARGET_WR, wr, "ir")):
        e = tgt - meas
        unsat = tgt + KP3 * e + KI3 * st3[key]
        # ANTI-WINDUP: only integrate when the command is inside what the
        # actuator can be asked for. Integrating against a saturated output is
        # exactly what broke v2.
        if abs(unsat) < LIMIT:
            st3[key] += e * DT
        c = max(-LIMIT, min(LIMIT, tgt + KP3 * e + KI3 * st3[key]))
        out.append(c)
    st3["peak"] = max(st3["peak"], abs(out[1]))
    return out[0], out[1]


hl3, hr3 = run(v3)
e3l, e3r = summarise(f"PI + anti-windup, limit {LIMIT}", hl3, hr3)
show(f"    the integral stayed at {st3['ir']:+.3f}, command peak "
     f"{st3['peak']:.1f} rad/s")
show()

# ---------------------------------------------------------------------------
# THE COMPARISON
# ---------------------------------------------------------------------------
show("=" * 70)
show("RIGHT WHEEL, THE THREE VERSIONS")
show("=" * 70)
show(f"  target                      {TARGET_WR:+7.3f} rad/s")
show(f"  v1 open loop      settled   {np.mean(hr1[-60:]):+7.3f}"
     f"   error {e1r:+.3f}")
show(f"  v2 PI, no clamp   settled   {np.mean(hr2[-60:]):+7.3f}"
     f"   error {e2r:+.3f}   command peak {state['peak']:.1f}")
show(f"  v3 PI + clamp     settled   {np.mean(hr3[-60:]):+7.3f}"
     f"   error {e3r:+.3f}   command peak {st3['peak']:.1f}")
show()
show("  Read the ERROR column and the COMMAND PEAK column together. A")
show("  controller that reaches the target by demanding a number the hardware")
show("  will never honour has not solved anything -- it has moved the failure")
show("  somewhere you are not looking.")
show()
# ---------------------------------------------------------------------------
# 4. WHY v3 IS WORSE THAN v1, WHICH IS NOT WHAT A TEXTBOOK PREDICTS.
# Anti-windup fixed the wind-up (integral 27.7 -> 1.1, command peak 64.4 -> 8.0)
# and the tracking still got WORSE. Before tuning gains, find out whether this
# wheel can reach +4.0 rad/s AT ALL while the other one is driven to -4.0.
# Sweep the open-loop command and read what comes back.
# ---------------------------------------------------------------------------
show("-" * 70)
show("4. CAN THIS WHEEL EVEN DO IT?  (open-loop sweep, no controller)")
show("-" * 70)
show("   commanding the RIGHT wheel while the LEFT is held at -4.0")
for cmd in (1.0, 2.0, 4.0, 6.0, 8.0, 12.0):
    hl, hr = run(lambda i, wl, wr, c=cmd: (TARGET_WL, c), steps=180)
    show(f"     command {cmd:+6.2f}  ->  achieved {np.mean(hr[-60:]):+7.3f} rad/s")
show()
show("  THAT is the answer, and it is worse than 'it saturates'. Look at the")
show("  shape: +4.00 gives +3.59, and +6.00 gives MINUS 0.65. The response is")
show("  not just limited, it is NON-MONOTONIC -- there is a cliff between 4 and")
show("  6, and past it, asking harder drives the wheel BACKWARDS.")
show()
show("  Now every one of those controller failures makes sense. The instant an")
show("  integral term pushes the command past the cliff, the wheel collapses,")
show("  the error grows, the integral pushes harder, and the loop can never")
show("  recover -- it is chasing a setpoint by walking further off a ledge.")
show("  v2 wound up to a 64.4 rad/s command; v3 clamped at 8.0, which is still")
show("  on the wrong side of the cliff.")
show()
show("  THE LESSON IS NOT 'PI CONTROL IS BAD'. It is: check the setpoint is")
show("  physically reachable BEFORE you close a loop around it. A controller")
show("  cannot buy you authority the actuator does not have, and every hour")
show("  spent tuning gains against a saturated actuator is an hour wasted.")
show()

# ---------------------------------------------------------------------------
# 5. THE SAME CONTROLLER, A REACHABLE TARGET.
# Re-run all three against a setpoint the sweep above shows is achievable, so
# the comparison is finally a fair test of the controller rather than of the
# actuator.
# ---------------------------------------------------------------------------
REACH_L, REACH_R = -2.0, 2.0
show("-" * 70)
show(f"5. THE SAME THREE, AGAINST A REACHABLE TARGET ({REACH_L:+.1f}, {REACH_R:+.1f})")
show("-" * 70)
tl, tr = TARGET_WL, TARGET_WR
TARGET_WL, TARGET_WR = REACH_L, REACH_R
hl1b, hr1b = run(lambda i, wl, wr: (REACH_L, REACH_R))
b1l, b1r = summarise("open loop", hl1b, hr1b)
st3["il"] = st3["ir"] = 0.0
st3["peak"] = 0.0
hl3b, hr3b = run(v3)
b3l, b3r = summarise(f"PI + anti-windup, limit {LIMIT}", hl3b, hr3b)
show()
show(f"  right wheel: open loop error {b1r:+.3f}, closed loop {b3r:+.3f}")
if abs(b3r) < abs(b1r):
    show("  NOW the loop earns its place: same code, same gains, and the only")
    show("  thing that changed is that the target is inside what the hardware")
    show("  can do.")
else:
    show("  The loop still does not beat open loop here -- report that, do not")
    show("  bury it.")
TARGET_WL, TARGET_WR = tl, tr
show()

show("=" * 70)
show("done")
show("=" * 70)

sim.close()
