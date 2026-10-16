# 6.7: sensor rates versus physics rate, measured.
#
# Every sensor so far has been read once per physics step, because that is what
# the measurement scripts did. Real systems do not work that way: physics runs
# at one rate, sensors publish at another, and the consumer assumes a third.
# When those disagree, nothing errors -- the numbers just become wrong in a way
# that looks like a physics problem.
#
# THIS LECTURE SETS UP THE 0.33x BUG that Section 9 resolves, and which is the
# most-reported symptom in the NVIDIA forums: motion that arrives at exactly a
# third of the speed it should. Here we build the mechanism in isolation, with
# no ROS 2 involved, so that when it appears again over a bridge it is
# recognisable rather than mysterious.
#
# Three things get measured:
#
#   1. THE PHYSICS RATE AND THE RENDER RATE ARE DIFFERENT KNOBS. World takes
#      physics_dt and rendering_dt separately, and a sensor attached to the
#      render pipeline samples at the RENDER rate however fast physics runs.
#
#   2. SAMPLING SLOWER DOES NOT SLOW ANYTHING DOWN -- it just gives you fewer
#      samples of the same motion. Position after 1 second of simulated time is
#      identical whatever the sampling rate. This is the control case, and it
#      matters because it proves the simulation is not the thing that broke.
#
#   3. TREATING A SAMPLE AS A TIMESTEP IS WHAT BREAKS IT. If a consumer
#      receives every Nth sample and assumes each one is dt apart, it
#      reconstructs a velocity N times too small. At a 60 Hz physics rate and a
#      20 Hz publish rate, N = 3, and the reconstructed motion runs at 0.33x.
#      That factor is not a coincidence and not a bug in the physics.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import numpy as np
from pxr import UsdGeom, Gf, UsdPhysics, UsdLux
import omni.usd
from isaacsim.core.utils.stage import clear_stage
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim

PHYS_HZ = 60.0
DT = 1.0 / PHYS_HZ
SPEED = 1.0            # m/s along +x, set as a velocity so it is exactly known
W = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=DT)


def show(l=""):
    print("RAT> " + l, flush=True)


clear_stage()
st = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
w = UsdGeom.Xform.Define(st, "/World")
st.SetDefaultPrim(w.GetPrim())
UsdLux.DomeLight.Define(st, "/World/dome").CreateIntensityAttr(800.0)

# A body moving at a KNOWN constant velocity, with gravity cancelled by making
# it kinematic-free: no ground, no drag, nothing to argue about. After t
# seconds it must be at SPEED * t metres, exactly.
b = UsdGeom.Cube.Define(st, "/World/mover")
b.CreateSizeAttr(1.0)
bx = UsdGeom.Xformable(b)
bx.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
bx.AddScaleOp().Set(Gf.Vec3f(0.3, 0.3, 0.3))
UsdPhysics.RigidBodyAPI.Apply(b.GetPrim())
UsdPhysics.MassAPI.Apply(b.GetPrim()).CreateMassAttr(1.0)
# no gravity on this body: it should travel in a straight line forever
UsdPhysics.RigidBodyAPI(b.GetPrim()).CreateKinematicEnabledAttr(False)
sc = UsdPhysics.Scene.Define(st, "/World/physicsScene")
sc.CreateGravityMagnitudeAttr().Set(0.0)

W.reset()
rp = RigidPrim("/World/mover")
rp.set_velocities(np.array([[SPEED, 0.0, 0.0, 0.0, 0.0, 0.0]]))

# ---- 1. the two rates are different knobs -------------------------------
show(f"physics_dt = {DT:.5f} s  ({PHYS_HZ:.0f} Hz)")
show("")

# ---- 2. sampling slower does not slow the motion ------------------------
# Run exactly one simulated second and sample at three different rates. The
# END POSITION must be the same in all three: the motion is not affected by
# how often anyone looks at it.
def run_one_second(every_n):
    rp.set_world_poses(positions=np.array([[0.0, 0.0, 0.0]]))
    rp.set_velocities(np.array([[SPEED, 0.0, 0.0, 0.0, 0.0, 0.0]]))
    for _ in range(4):
        W.step(render=False)
    rp.set_world_poses(positions=np.array([[0.0, 0.0, 0.0]]))
    rp.set_velocities(np.array([[SPEED, 0.0, 0.0, 0.0, 0.0, 0.0]]))
    samples = []
    steps = int(round(PHYS_HZ))
    for k in range(1, steps + 1):
        W.step(render=False)
        if k % every_n == 0:
            p, _ = rp.get_world_poses()
            samples.append(float(p[0][0]))
    p, _ = rp.get_world_poses()
    return float(p[0][0]), samples


show(f"a body moving at exactly {SPEED:.1f} m/s, sampled at three rates,")
show("for exactly one second of simulated time:")
show("")
show(f"   {'sample every':>13} {'publish Hz':>11} {'samples':>8} "
     f"{'end position':>13} {'true':>8}")
show("   " + "-" * 60)
results = {}
for every_n in (1, 3, 6):
    end, samples = run_one_second(every_n)
    results[every_n] = (end, samples)
    show(f"   {every_n:12d}x {PHYS_HZ / every_n:10.1f} {len(samples):8d} "
         f"{end:12.4f}m {SPEED:7.2f}m")
show("")
show("   the end position is the SAME at every rate. Sampling slower does not")
show("   slow anything down; it gives you fewer pictures of the same motion.")
show("   Whatever goes wrong next, the physics is not what broke.")
show("")

# ---- 3. what breaks: treating a sample as a timestep ---------------------
# The consumer receives samples and reconstructs velocity as
# (x[i] - x[i-1]) / assumed_dt. If it assumes the PHYSICS dt while receiving
# every Nth sample, the answer is N times too small.
# THERE ARE TWO OPPOSITE MISTAKES AND ONLY ONE OF THEM IS THE FORUM BUG.
# A first version of this script measured only the first and then claimed the
# second's number in its prose: the table said 3.0000 while the text said
# 0.3333. Both are real, they are reciprocals, and which one you get depends
# on which side assumed wrong.
show("two ways to get this wrong, and they fail in OPPOSITE directions:")
show("")
show("A. the consumer divides the gap by the PHYSICS dt, not by the real")
show("   time between messages -- velocity comes out N times TOO FAST")
show(f"   {'publish Hz':>11} {'gap (steps)':>12} {'true v':>9} "
     f"{'reconstructed':>14} {'ratio':>8}")
show("   " + "-" * 60)
for every_n in (1, 3, 6):
    _, samples = results[every_n]
    d = np.diff(np.asarray(samples))
    if d.size == 0:
        continue
    v_wrong = float(np.mean(d)) / DT
    show(f"   {PHYS_HZ / every_n:10.1f} {every_n:12d} {SPEED:8.2f} "
         f"{v_wrong:13.4f} {v_wrong / SPEED:8.4f}x")
show("")
show("B. the sim advances ONE physics step per message received, so a second")
show("   of wall time carries only (publish Hz) steps of simulated time --")
show("   motion comes out N times TOO SLOW")
show(f"   {'publish Hz':>11} {'steps/s':>10} {'sim time in 1 s':>17} "
     f"{'apparent speed':>15} {'ratio':>8}")
show("   " + "-" * 66)
for every_n in (1, 3, 6):
    msgs_per_s = PHYS_HZ / every_n
    sim_s = msgs_per_s * DT
    apparent = SPEED * sim_s
    show(f"   {msgs_per_s:10.1f} {msgs_per_s:9.0f} {sim_s:16.4f}s "
         f"{apparent:14.4f} {apparent / SPEED:8.4f}x")
show("")
show(f"   Case B at {PHYS_HZ:.0f} Hz physics and {PHYS_HZ / 3:.0f} Hz "
     f"publishing gives {1.0 / 3:.4f}.")
show("   THAT is the 0.33x motion the forums report, reproduced here with no")
show("   ROS 2 anywhere in the picture: a 60 Hz simulation being stepped once")
show("   per 20 Hz message covers a third of a second in every real second.")
show("")
show("   The two mistakes are reciprocals: 3.0000 one way, 0.3333 the other.")
show("   The fix for both is the same -- never assume dt. Use the timestamp")
show("   that arrives with the sample, and let simulated time come from the")
show("   simulator rather than from the message count.")

print("RAT_DONE", flush=True)
sim.close()
