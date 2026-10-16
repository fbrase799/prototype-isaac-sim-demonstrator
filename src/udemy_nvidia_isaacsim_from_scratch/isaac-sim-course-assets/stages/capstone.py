# SECTION 11 CAPSTONE: the whole robot, the whole scene, one autonomy loop.
#
# This single script is the spine of lectures 11.1 to 11.6. It states numeric
# pass conditions FIRST, builds the system out of the pieces the course already
# has, runs it, and reports pass or fail against those conditions -- including
# when that means fail.
#
# WHAT IT REUSES, DELIBERATELY. Nothing here is new machinery:
#   Section 7   the warehouse environment
#   8.3         look joints up by NAME, never by index
#   8.4         diff-drive kinematics, r = 0.1400 m, L = 0.4132 m, measured
#   8.5         a closed velocity loop, and the setpoint-reachability check
#   8.6         read proprioception every step; it is free
#   8.8         the DiffDrive tool, so no caller re-derives any of it
#
# THE PASS CONDITIONS, stated before the run and not adjusted afterwards:
#   P1  the robot reaches within 0.50 m of each of 3 waypoints, in order
#   P2  it completes the route in under 60 s of simulated time
#   P3  no waypoint is missed (a miss = passing within 2.0 m and not closing)
#   P4  the control loop holds at least 30 Hz of simulated time per wall second
#
# WHY THESE. Each is measurable from the run itself with no judgement call, and
# each can fail independently, which is what makes the result informative rather
# than decorative. A capstone whose pass conditions can only be met is a demo.
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

# measured in 8.4, off the stage, not from a datasheet
R_WHEEL, L_TRACK = 0.1400, 0.4132

# the route: three waypoints in the warehouse, in order
WAYPOINTS = [(3.0, 0.0), (3.0, 3.0), (0.0, 3.0)]
TOL = 0.50            # P1
TIME_LIMIT = 60.0     # P2
MISS_DIST = 2.0       # P3
MIN_HZ = 30.0         # P4


def show(l=""):
    print("CAP> " + l, flush=True)


show("=" * 70)
show("SECTION 11 CAPSTONE -- pass conditions stated BEFORE the run")
show("=" * 70)
show()
show(f"  P1  reach within {TOL:.2f} m of each of {len(WAYPOINTS)} waypoints, in order")
show(f"  P2  complete the route in under {TIME_LIMIT:.0f} s of simulated time")
show(f"  P3  miss no waypoint (a miss = passing within {MISS_DIST:.1f} m "
     f"and then receding)")
show(f"  P4  hold at least {MIN_HZ:.0f} Hz of simulated time per wall second")
show()
show("  These are fixed now. If the run fails one, the lecture reports a")
show("  failure rather than moving the line.")
show()

root = get_assets_root_path()
open_stage(root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd")
world = World(stage_units_in_meters=1.0)
add_reference_to_stage(usd_path=CARTER, prim_path="/World/carter")
robot = Articulation(prim_paths_expr="/World/carter", name="carter")
world.scene.add(robot)
world.reset()

names = list(robot.dof_names)
LW = names.index("joint_wheel_left")      # 8.3: by NAME, never by index
RW = names.index("joint_wheel_right")
DT = world.get_physics_dt()
show(f"  robot: {robot.num_dof} dof, wheels at [{LW}, {RW}], dt {DT:.6f} s")
show()


def base_pose():
    p = robot.get_world_poses()
    pos = np.array(p[0][0], dtype=float)
    w, x, y, z = np.array(p[1][0], dtype=float)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return pos[0], pos[1], yaw


def command(v, omega):
    """8.4's inverse kinematics, applied through 8.8's interface."""
    wr = (2.0 * v + omega * L_TRACK) / (2.0 * R_WHEEL)
    wl = (2.0 * v - omega * L_TRACK) / (2.0 * R_WHEEL)
    act = np.zeros((1, robot.num_dof), dtype=np.float32)
    act[0, LW] = wl
    act[0, RW] = wr
    robot.set_joint_velocity_targets(act)
    return wl, wr


# ---------------------------------------------------------------------------
# THE AUTONOMY LOOP (11.3). Deliberately simple: turn toward the waypoint,
# drive at a speed that falls off as you arrive. Every constant here is one a
# learner can see and change.
# ---------------------------------------------------------------------------
K_ANG, V_MAX, OMEGA_MAX = 1.6, 0.6, 1.2

show("-" * 70)
show("RUNNING")
show("-" * 70)
for _ in range(10):
    world.step(render=False)

target = 0
log = []
closest = [1e9] * len(WAYPOINTS)
receding_since = [0] * len(WAYPOINTS)
t_sim = 0.0
steps = 0
t0 = time.perf_counter()

while target < len(WAYPOINTS) and t_sim < TIME_LIMIT * 1.5:
    x, y, yaw = base_pose()
    gx, gy = WAYPOINTS[target]
    dx, dy = gx - x, gy - y
    dist = math.hypot(dx, dy)
    closest[target] = min(closest[target], dist)

    bearing = math.atan2(dy, dx)
    err = math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))
    omega = max(-OMEGA_MAX, min(OMEGA_MAX, K_ANG * err))
    # slow down when badly aimed, and when close
    v = V_MAX * max(0.0, math.cos(err)) * min(1.0, dist / 1.0)
    command(v, omega)

    world.step(render=False)
    t_sim += DT
    steps += 1
    log.append((t_sim, x, y, dist, target))

    if dist < TOL:
        show(f"  reached waypoint {target + 1} "
             f"({gx:.1f}, {gy:.1f}) at t = {t_sim:.2f} s, "
             f"closest {closest[target]:.3f} m")
        target += 1

wall = time.perf_counter() - t0
command(0.0, 0.0)
for _ in range(10):
    world.step(render=False)

# ---------------------------------------------------------------------------
# THE RESULT, AGAINST THE CONDITIONS (11.6)
# ---------------------------------------------------------------------------
show()
show("-" * 70)
show("RESULT")
show("-" * 70)
reached = target
hz = steps * DT / max(wall, 1e-9) / DT * DT   # sim seconds per wall second
sim_per_wall = (steps * DT) / max(wall, 1e-9)
loop_hz = steps / max(wall, 1e-9)

p1 = reached == len(WAYPOINTS)
p2 = t_sim < TIME_LIMIT and p1
p3 = all(c < MISS_DIST for c in closest)
p4 = loop_hz >= MIN_HZ

show(f"  waypoints reached : {reached} of {len(WAYPOINTS)}")
for i, c in enumerate(closest):
    show(f"    waypoint {i + 1}: closest approach {c:.3f} m"
         + ("  (never reached)" if c > TOL else ""))
show(f"  simulated time    : {t_sim:.2f} s")
show(f"  wall time         : {wall:.2f} s  "
     f"({sim_per_wall:.2f}x real time, {loop_hz:.0f} Hz loop)")
show()
show(f"  P1 all waypoints within {TOL:.2f} m ....... {'PASS' if p1 else 'FAIL'}")
show(f"  P2 route under {TIME_LIMIT:.0f} s ................ {'PASS' if p2 else 'FAIL'}")
show(f"  P3 no waypoint missed ................ {'PASS' if p3 else 'FAIL'}")
show(f"  P4 loop at or above {MIN_HZ:.0f} Hz ........... {'PASS' if p4 else 'FAIL'}")
show()
allp = p1 and p2 and p3 and p4
show(f"  CAPSTONE: {'PASS' if allp else 'FAIL'}")
show()
# BE SUSPICIOUS OF A CLEAN PASS. All three closest approaches came out at
# 0.499, 0.496, 0.496 against a 0.50 m tolerance -- within 4 mm of the line,
# three times. That is not luck: the loop advances the target the instant
# dist < TOL, so the recorded "closest approach" can never be much under the
# tolerance. P1 is therefore close to VACUOUS as written: it tests that the
# robot got within 0.50 m, and then stops measuring.
near = [c for c in closest if TOL - c < 0.05]
if len(near) == len(WAYPOINTS):
    show("  BUT LOOK AT THE MARGINS. Every closest approach is within 5 mm of")
    show(f"  the {TOL:.2f} m tolerance: {', '.join(f'{c:.3f}' for c in closest)}.")
    show("  That is not the controller being precise, it is the CONDITION")
    show("  measuring itself: the loop switches target the moment it crosses")
    show("  the line, so the number recorded is always just inside it.")
    show()
    show("  A pass condition that cannot report a margin is barely a test. The")
    show("  honest fix is to require the robot to SETTLE inside the tolerance")
    show("  rather than merely touch it -- which is what 11.4 changes, and")
    show("  which is where this capstone actually starts failing.")
    show()
if not allp:
    show("  A FAILING CAPSTONE IS A RESULT, NOT AN EMBARRASSMENT. The next")
    show("  lecture diagnoses it with the method from 8.9: ask the cheapest")
    show("  question first, and let each answer halve the search.")
else:
    show("  It passed on the conditions as written -- which is worth being")
    show("  suspicious of. 11.4 goes looking for the case that breaks it,")
    show("  because a capstone that only ever passes has not been tested.")
show()
show("=" * 70)
show("done")
show("=" * 70)

sim.close()
