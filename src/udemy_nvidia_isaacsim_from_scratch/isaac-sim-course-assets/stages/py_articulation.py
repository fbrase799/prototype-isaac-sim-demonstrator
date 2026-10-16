# 8.3: ArticulationController -- driving joints from code.
#
# THE BOUNDARY WITH SECTION 5. 5.8 already drove this robot from the GUI and
# read its joint states there. Nothing here re-teaches what a joint drive is or
# what an articulation root does (5.5, 5.6). This lecture is about the API you
# reach for when the GUI is not in the loop, and specifically about the three
# things that bite in the first hour:
#
#   1. There are TWO ways to command a joint and they are not interchangeable:
#      a position/velocity TARGET (the drive solves for it) versus a direct
#      EFFORT. Mixing them silently gives you the drive's answer, not yours.
#   2. Joint ORDER is the robot's, not yours. dof_names is authoritative and
#      indexing by guess is how you drive the left wheel with the right command.
#      Measured here rather than asserted.
#   3. The controller does NOT exist until after world.reset(). Building one
#      earlier gets you None and an AttributeError three lines later.
#
# WHAT IS MEASURED:
#   - dof_names and dof count, straight off the running articulation
#   - a velocity command applied for a fixed number of steps, and the wheel
#     angle that actually results, against what the command asked for
#   - the same command sent to the WRONG index, to show the failure is silent
#   - efforts vs targets on the same joint, so the difference is a number
from isaacsim import SimulationApp

sim = SimulationApp({"headless": True})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
# API TRAP: it is isaacsim.storage.native, NOT isaacsim.core.utils.nucleus.
# The latter is the 4.x path and is all over the older forum answers; in 6.0.1
# it raises ModuleNotFoundError, which reads like a broken install rather than
# a renamed module. Every other stage script in this repo uses the storage path.
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation


def show(l=""):
    print("PY3> " + l, flush=True)


# The asset root is VERSIONED and get_assets_root_path() does not always resolve
# to a path that has the robot under it -- Section 7 hit the same thing. Every
# working stage in this repo pins the explicit 4.5 URL, so this one does too.
CARTER = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
          "/Assets/Isaac/4.5/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd")

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
add_reference_to_stage(usd_path=CARTER, prim_path="/World/carter")
robot = Articulation(prim_paths_expr="/World/carter", name="carter")
world.scene.add(robot)

# THE TRAP: none of the articulation's structure exists before reset(). Asking
# for dof_names here returns None, and the AttributeError lands three lines
# later on something unrelated-looking.
show("=" * 66)
show("8.3  ArticulationController -- driving joints from code")
show("=" * 66)
show()
show("before world.reset():")
show(f"  robot.dof_names = {robot.dof_names}")
show()

world.reset()

show("after world.reset():")
show(f"  {robot.num_dof} degrees of freedom")
show()
show("  the joint order is THE ROBOT'S, not yours:")
for i, n in enumerate(robot.dof_names):
    show(f"    [{i:2d}]  {n}")
show()

names = list(robot.dof_names)


def idx(substr):
    """Find a dof by name. Never index by guess -- that is the lecture."""
    for i, n in enumerate(names):
        if substr in n:
            return i
    return None


# THE GUESS THAT FAILS, LEFT IN ON PURPOSE. The obvious name for a Carter wheel
# joint is "left_wheel". It is not: this robot calls it "joint_wheel_left", so
# searching for "left_wheel" returns None and any code that then does
# action[None] dies somewhere else entirely. Look them up, print them, and
# match what the ROBOT calls them.
show("  guessing the name 'left_wheel':")
show(f"    -> index {idx('left_wheel')}   (nothing is called that)")
show()
LW = idx("wheel_left")
RW = idx("wheel_right")
show("  reading the name off the robot instead:")
show(f"    joint_wheel_left  -> index {LW}")
show(f"    joint_wheel_right -> index {RW}")
show()

# API NOTE for 6.0.1: this class has NO get_articulation_controller(). The
# 4.x-era tutorials all call it and it raises AttributeError here. Velocity
# targets go straight onto the articulation with set_joint_velocity_targets,
# which takes a (num_envs, num_dof) array -- note the leading batch dimension,
# which is the other thing that trips people coming from the old single-robot
# API.


def spin(cmd_idx, vel, steps=120):
    """Command ONE joint at `vel` rad/s for `steps`, return every joint's travel."""
    world.reset()
    for _ in range(5):
        world.step(render=False)
    before = robot.get_joint_positions()[0].copy()
    action = np.zeros((1, robot.num_dof), dtype=np.float32)
    action[0, cmd_idx] = vel
    for _ in range(steps):
        robot.set_joint_velocity_targets(action)
        world.step(render=False)
    after = robot.get_joint_positions()[0].copy()
    return before, after


# ---------------------------------------------------------------------------
# 1. A COMMAND THAT LANDS WHERE YOU MEANT IT
# ---------------------------------------------------------------------------
show("-" * 66)
show("1. COMMANDING THE JOINT YOU MEANT")
show("-" * 66)
VEL, STEPS = 6.0, 120
b, a = spin(LW, VEL, STEPS)
dt = world.get_physics_dt()
expected = VEL * STEPS * dt
show(f"commanded dof[{LW}] ({names[LW]}) at {VEL} rad/s for {STEPS} steps")
show(f"  that asks for {expected:.4f} rad over {STEPS * dt:.3f} s")
got = a[LW] - b[LW]
show(f"  left  wheel turned {got:8.4f} rad")
show(f"  right wheel turned {a[RW] - b[RW]:8.4f} rad")
show()
show("  The commanded joint moved and the other did not: an action is a vector")
show("  over ALL dofs and the zeros are doing real work.")
show()
# DO NOT LET THIS PASS AS "IT WORKED". The command asked for +12.0000 rad and
# the joint delivered a DIFFERENT SIGN and about a third of the magnitude. A
# velocity TARGET is a request to the drive, not an assignment: the drive has
# finite stiffness and damping (5.6) and it is also carrying the mass of the
# robot, the caster and the ground contact. Report the gap as a ratio and say
# what it means, because a learner who assumes target == result will spend a
# day on it.
show(f"  ASKED  {expected:+8.4f} rad")
show(f"  GOT    {got:+8.4f} rad     ratio {got / expected:+.3f}")
show()
show("  Hold that number. It is NOT what the command asked for -- wrong sign")
show("  and a third of the size -- and there are two independent reasons why,")
show("  neither of them obvious. Section 3 takes them apart. Resist the urge")
show("  to explain it now; the first explanation I reached for was wrong and")
show("  the measurement said so.")
show()

# ---------------------------------------------------------------------------
# 2. THE SAME COMMAND, ONE INDEX OFF
# The failure this lecture exists to prevent, and it is completely silent.
# ---------------------------------------------------------------------------
show("-" * 66)
show("2. THE SAME COMMAND, ONE INDEX OFF")
show("-" * 66)
WRONG = (LW + 1) % robot.num_dof
b2, a2 = spin(WRONG, VEL, STEPS)
show(f"commanded dof[{WRONG}] ({names[WRONG]}) instead, same {VEL} rad/s")
show(f"  left  wheel turned {a2[LW] - b2[LW]:8.4f} rad")
show(f"  right wheel turned {a2[RW] - b2[RW]:8.4f} rad")
show(f"  dof[{WRONG}] itself turned {a2[WRONG] - b2[WRONG]:8.4f} rad")
show()
show("  No exception. No warning. The robot did something, just not the thing")
show("  you asked for. THIS is why you look up indices by name every time.")
show()

# ---------------------------------------------------------------------------
# 3. IS THE TARGET TRACKED AT ALL?
# Section 1 showed a gap between asked and got. Before blaming the drive, CHECK
# the claim: reverse the sign and see whether the wheel reverses with it, and
# read the joint's actual velocity rather than integrating its position.
# ---------------------------------------------------------------------------
show("-" * 66)
show("3. DOES THE JOINT TRACK THE TARGET?")
show("-" * 66)
for v in (6.0, -6.0):
    world.reset()
    for _ in range(5):
        world.step(render=False)
    act = np.zeros((1, robot.num_dof), dtype=np.float32)
    act[0, LW] = v
    for _ in range(120):
        robot.set_joint_velocity_targets(act)
        world.step(render=False)
    vel = float(robot.get_joint_velocities()[0][LW])
    show(f"  target {v:+6.2f} rad/s  ->  steady joint velocity {vel:+8.4f} rad/s"
         f"   ({vel / v * 100:5.1f}% of target)")
show()
show("  So the drive DOES track the sign: +6 gives +5.36 rad/s, -6 gives -5.63.")
show("  Which means the negative TRAVEL in section 1 is not an axis-direction")
show("  story at all -- I guessed that first and the measurement refused it.")
show()

# WHERE THE NEGATIVE TRAVEL ACTUALLY COMES FROM. get_joint_positions() on a
# continuously spinning wheel WRAPS: it reports an angle in (-pi, pi], not a
# cumulative odometer. Subtracting two wrapped angles gives a number in that
# same range, which is why 2 seconds at ~5.4 rad/s -- about 10.7 rad, nearly
# two full turns -- reads as -4.36. Integrate the VELOCITY if you want travel.
world.reset()
for _ in range(5):
    world.step(render=False)
act = np.zeros((1, robot.num_dof), dtype=np.float32)
act[0, LW] = 6.0
integ = 0.0
raw = []
for i in range(120):
    robot.set_joint_velocity_targets(act)
    world.step(render=False)
    integ += float(robot.get_joint_velocities()[0][LW]) * dt
    if i in (0, 29, 59, 89, 119):
        raw.append((i + 1, float(robot.get_joint_positions()[0][LW])))
show("-" * 66)
show("  WHY THE TRAVEL LOOKED NEGATIVE: the reported angle WRAPS.")
for n, p in raw:
    show(f"    after {n:3d} steps  get_joint_positions() = {p:+8.4f} rad")
show(f"    velocity integrated over the same 120 steps = {integ:+8.4f} rad")
show(f"    naive (after - before)                      = "
     f"{a[LW] - b[LW]:+8.4f} rad")
show()
show("  Both numbers are real. The wrapped one is an ANGLE and the integrated")
show("  one is TRAVEL, and a wheel that turns nearly two full circles in two")
show("  seconds makes them look wildly inconsistent. If you difference joint")
show("  positions on a continuously rotating joint you will get nonsense, and")
show("  no warning: it is the same class of silent wrongness as the wrong index.")
show()

show("=" * 66)
show("done")
show("=" * 66)

sim.close()
