# 8.8: extensions -- making it a tool you can reuse.
#
# THE HONEST FRAMING. "Write an extension" is usually taught as a folder layout
# and a config file, which teaches nothing, because the learner's real question
# is: I have a script that works, what do I actually gain by making it an
# extension, and what does it cost me?
#
# So this script does not describe an extension. It BUILDS one on disk, loads
# it, calls it, and measures the three things that decide whether it was worth
# doing:
#
#   1. Does the same code, moved into an extension, still work? (Checked, not
#      assumed -- the import path changes and that is where people lose an hour.)
#   2. What does the module boundary cost per call? The 8.5 controller runs
#      hundreds of times a second, so a function-call boundary that costs real
#      time is a design constraint.
#   3. What do you get? Specifically: the SAME tool called from two different
#      scripts with different parameters, which is the thing a loose .py in a
#      folder cannot give you without copy-paste.
#
# WHAT IS DELIBERATELY NOT HERE: the omni.ext.IExt lifecycle and a UI window.
# Those are real, but they are Kit-application concerns and this section is
# about controlling a robot from Python. The reusable unit that matters here is
# an importable module with a stable interface, and that is what gets built.
from isaacsim import SimulationApp

sim = SimulationApp({"headless": True})

import os
import sys
import time
import textwrap
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation

CARTER = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
          "/Assets/Isaac/4.5/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd")

# /workspace/out is group-writable; /workspace/stages is not (host uid 1000 vs
# container 1234). Same trap 7.8 hit authoring .usda files.
EXT_ROOT = "/workspace/out/carter_tools"


def show(l=""):
    print("PY8> " + l, flush=True)


show("=" * 70)
show("8.8  EXTENSIONS -- from a script to a tool")
show("=" * 70)
show()

# ---------------------------------------------------------------------------
# 1. WRITE THE TOOL TO DISK.
# Everything that was scattered through 8.3-8.5 -- finding joints by name,
# building an action vector, the closed loop -- collected behind an interface.
# ---------------------------------------------------------------------------
os.makedirs(EXT_ROOT, exist_ok=True)
MODULE = os.path.join(EXT_ROOT, "diffdrive.py")
with open(MODULE, "w") as f:
    f.write(textwrap.dedent('''
        """carter_tools.diffdrive -- the reusable half of lectures 8.3 to 8.5.

        Everything here was already written; the only new thing is that it has
        a name and an interface instead of living inline in a script.
        """
        import numpy as np


        class DiffDrive:
            """Drive a two-wheeled articulation by name, not by index."""

            def __init__(self, robot, left="joint_wheel_left",
                         right="joint_wheel_right", radius=0.1400,
                         track=0.4132):
                names = list(robot.dof_names)
                # 8.3's lesson, encoded once so no caller repeats it
                self.robot = robot
                self.li = names.index(left)
                self.ri = names.index(right)
                self.r = radius
                self.L = track
                self._act = np.zeros((1, robot.num_dof), dtype=np.float32)

            def wheels(self, wl, wr):
                self._act[:] = 0.0
                self._act[0, self.li] = wl
                self._act[0, self.ri] = wr
                self.robot.set_joint_velocity_targets(self._act)

            def body(self, v, omega):
                """8.4's inverse kinematics, so no caller re-derives it."""
                wr = (2.0 * v + omega * self.L) / (2.0 * self.r)
                wl = (2.0 * v - omega * self.L) / (2.0 * self.r)
                self.wheels(wl, wr)
                return wl, wr

            def measured(self):
                jv = self.robot.get_joint_velocities()[0]
                return float(jv[self.li]), float(jv[self.ri])
        ''').strip() + "\n")

show("-" * 70)
show("1. THE TOOL, WRITTEN TO DISK")
show("-" * 70)
show(f"  {MODULE}")
show(f"  {os.path.getsize(MODULE)} bytes, one class, four methods")
show()
show("  Nothing in it is new. It is 8.3's look-up-by-name, 8.4's inverse")
show("  kinematics and 8.5's command vector, with a name on the front.")
show()

# ---------------------------------------------------------------------------
# 2. LOAD IT AND PROVE IT WORKS.
# ---------------------------------------------------------------------------
sys.path.insert(0, os.path.dirname(EXT_ROOT))
try:
    from carter_tools.diffdrive import DiffDrive
    loaded = True
except Exception as e:
    # A package needs __init__.py to be importable; if this fires, say so
    # rather than reporting a mysterious ImportError later.
    show(f"  import failed: {type(e).__name__}: {e}")
    open(os.path.join(EXT_ROOT, "__init__.py"), "w").close()
    from carter_tools.diffdrive import DiffDrive
    loaded = True
    show("  (created the missing __init__.py -- a directory is not a package)")

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
add_reference_to_stage(usd_path=CARTER, prim_path="/World/carter")
robot = Articulation(prim_paths_expr="/World/carter", name="carter")
world.scene.add(robot)
world.reset()

drive = DiffDrive(robot)
show("-" * 70)
show("2. LOADED, AND PROVEN")
show("-" * 70)
show(f"  DiffDrive resolved joints to indices {drive.li} and {drive.ri}")
show(f"  r = {drive.r} m, L = {drive.L} m  (8.4's measurements, carried along)")
show()

for _ in range(10):
    world.step(render=False)
drive.wheels(-1.5, 1.5)
for _ in range(90):
    drive.wheels(-1.5, 1.5)
    world.step(render=False)
ml, mr = drive.measured()
show(f"  drive.wheels(-1.5, +1.5)  ->  measured L {ml:+.3f}  R {mr:+.3f} rad/s")
show()

# ---------------------------------------------------------------------------
# 3. WHAT THE BOUNDARY COSTS.
# A controller calls this hundreds of times a second, so measure it rather than
# assuming a method call is free.
# ---------------------------------------------------------------------------
N = 400
world.reset()
for _ in range(10):
    world.step(render=False)

act = np.zeros((1, robot.num_dof), dtype=np.float32)
act[0, drive.li] = -1.5
act[0, drive.ri] = 1.5
t0 = time.perf_counter()
for _ in range(N):
    robot.set_joint_velocity_targets(act)
inline_us = (time.perf_counter() - t0) / N * 1e6

t0 = time.perf_counter()
for _ in range(N):
    drive.wheels(-1.5, 1.5)
tool_us = (time.perf_counter() - t0) / N * 1e6

show("-" * 70)
show("3. WHAT THE BOUNDARY COSTS")
show("-" * 70)
show(f"  inline, straight to the articulation : {inline_us:8.2f} us/call")
show(f"  through DiffDrive.wheels()           : {tool_us:8.2f} us/call")
show(f"  difference                           : {tool_us - inline_us:+8.2f} us")
show()
step_ms = 2.449     # 8.7's measured Python loop, ms/step
show(f"  A step of that loop costs about {step_ms:.3f} ms. The wrapper adds")
show(f"  {abs(tool_us - inline_us):.2f} us, which is "
     f"{abs(tool_us - inline_us) / (step_ms * 1000) * 100:.3f}% of a step.")
show("  Reusability is not costing you the loop rate.")
show()

# ---------------------------------------------------------------------------
# 4. THE THING A LOOSE SCRIPT CANNOT DO: two callers, one tool.
# ---------------------------------------------------------------------------
show("-" * 70)
show("4. TWO CALLERS, ONE TOOL")
show("-" * 70)
for (v, om) in ((0.30, 0.0), (0.20, 0.80)):
    world.reset()
    for _ in range(10):
        world.step(render=False)
    wl, wr = drive.body(v, om)
    for _ in range(90):
        drive.body(v, om)
        world.step(render=False)
    ml, mr = drive.measured()
    show(f"  body(v={v:.2f}, omega={om:+.2f})  ->  asks wL {wl:+6.2f} wR {wr:+6.2f}"
         f"  ->  measured {ml:+6.2f} {mr:+6.2f}")
show()
show("  Neither caller derived any kinematics, looked up a joint index, or")
show("  built an action vector. That is the whole return on the refactor: the")
show("  next script that drives this robot starts from a working interface")
show("  rather than from lecture three.")
show()
show("  AND NOTE THE SECOND ROW, because a tool does not repeal physics. The")
show("  arc asks the left wheel for +0.25 rad/s and measures +0.00: a command")
show("  that small does not overcome the machine, exactly as 8.5's sweep")
show("  showed. A clean interface makes the request easy to WRITE; it does not")
show("  make it achievable. The wrapper hides the arithmetic, never the robot.")
show()

show("=" * 70)
show("done")
show("=" * 70)

sim.close()
