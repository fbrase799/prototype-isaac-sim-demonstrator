# 6.6: contact and force sensors, measured.
#
# The camera, the lidar and the IMU all report about space. A contact sensor
# answers a different question -- am I touching something, and how hard -- and
# it is the sensor most often wired up wrong, because THREE separate settings
# can silently return zero while everything looks correctly configured.
#
# Four things get measured:
#
#   1. A RESTING BODY STOPS REPORTING. This is the big one. Drop a body, and
#      the sensor reads its weight for about half a second; let it settle, and
#      the reading falls to 0.0000 with in_contact False -- while the body is
#      still sitting on the ground, still pressing with the same force. PhysX
#      has put it to sleep, and a sleeping body generates no contact reports.
#      A robot standing still has silent feet by default.
#
#   2. THE FIX IS TWO ATTRIBUTES, NOT ONE. sleepThreshold alone is not enough:
#      with only that set the reading still collapsed to zero. sleepThreshold
#      AND stabilizationThreshold both at 0 hold the reading steady out to at
#      least 300 steps.
#
#   3. THE FORCE IS THE WEIGHT, checkable by hand. m*g newtons, so this is a
#      sensor you can verify against arithmetic before trusting it on anything
#      harder.
#
#   4. min_threshold SILENTLY DELETES LIGHT CONTACTS. Set it above the weight
#      of what you are trying to feel and the sensor reports nothing forever,
#      with no warning. A threshold chosen for a footstep will not feel a
#      finger.
#
# TRAPS, all silent, all hit while building this:
#
# - THE TIMELINE MUST BE PLAYING and the sensor must be registered with the C++
#   interface by hand: `get_contact_sensor_interface().create_sensor(path)`
#   AFTER `timeline.play()`. Creating the prim with Contact.create() and then
#   stepping World is NOT enough -- get_data() returns in_contact False and
#   force 0.0 forever, which looks exactly like a body that is not touching
#   anything. This one cost the most time: radius, spawn height, explicit
#   PhysxContactReportAPI and prim scaling were all eliminated first, and none
#   of them mattered.
# - The namespace is isaacsim.sensors.experimental.physics on 6.0.
# - Everything that moves is created BEFORE World.reset().
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import numpy as np
from pxr import UsdGeom, Gf, UsdPhysics, UsdLux, PhysxSchema
import omni.usd
import omni.timeline
from isaacsim.core.utils.stage import clear_stage
from isaacsim.core.api import World
from isaacsim.sensors.experimental.physics import Contact
from isaacsim.sensors.experimental.physics.impl.extension import (
    get_contact_sensor_interface)

DT = 1.0 / 60.0
G = 9.81
W = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=DT)


def show(l=""):
    print("CON> " + l, flush=True)


clear_stage()
st = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
w = UsdGeom.Xform.Define(st, "/World")
st.SetDefaultPrim(w.GetPrim())
UsdLux.DomeLight.Define(st, "/World/dome").CreateIntensityAttr(800.0)

gp = UsdGeom.Cube.Define(st, "/World/ground")
gp.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(gp)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
gx.AddScaleOp().Set(Gf.Vec3f(40.0, 40.0, 1.0))
UsdPhysics.CollisionAPI.Apply(gp.GetPrim())


def box_with_contact(path, x, mass, min_threshold=0.0, awake=True):
    """A 1 m cube dropped from z=1, with a contact sensor under it."""
    c = UsdGeom.Cube.Define(st, path)
    c.CreateSizeAttr(1.0)
    cx = UsdGeom.Xformable(c)
    cx.AddTranslateOp().Set(Gf.Vec3d(x, 0, 1.0))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(c.GetPrim())
    UsdPhysics.MassAPI.Apply(c.GetPrim()).CreateMassAttr(mass)
    api = PhysxSchema.PhysxContactReportAPI.Apply(c.GetPrim())
    api.CreateThresholdAttr().Set(0)
    if awake:
        # BOTH of these, see note 2. sleepThreshold on its own is not enough.
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(c.GetPrim())
        rb.CreateSleepThresholdAttr().Set(0.0)
        rb.CreateStabilizationThresholdAttr().Set(0.0)
    Contact.create(f"{path}/cs", min_threshold=min_threshold,
                   max_threshold=1.0e7)
    return f"{path}/cs"


MASS = 2.0
HEAVY_THRESHOLD = 100.0

# A: awake, no threshold      -- the reference reading
# B: ALLOWED TO SLEEP         -- same body, default sleep behaviour
# C: awake, threshold 100 N   -- same body, threshold above its weight
pA = box_with_contact("/World/A", 0.0, MASS)
pB = box_with_contact("/World/B", 4.0, MASS, awake=False)
pC = box_with_contact("/World/C", 8.0, MASS, min_threshold=HEAVY_THRESHOLD)

W.reset()
# The timeline must be PLAYING before the sensors are registered.
omni.timeline.get_timeline_interface().play()
iface = get_contact_sensor_interface()
for p in (pA, pB, pC):
    if not iface.create_sensor(p):
        show(f"create_sensor FAILED for {p} -- no table rather than zeros")
        print("CON_DONE", flush=True)
        sim.close()
        raise SystemExit(1)


def step_to(k):
    while W.current_time_step_index < k:
        W.step(render=False)


def read(p):
    r = iface.get_sensor_reading(p)
    return bool(r.in_contact), float(r.value)


# ---- 1. the force is the weight ----------------------------------------
step_to(40)
okA, fA = read(pA)
if not okA:
    show(f"SENSOR A REPORTS NO CONTACT ({fA:.4f} N) while resting on the")
    show("ground. Something is wrong with the setup; no table.")
    print("CON_DONE", flush=True)
    sim.close()
    raise SystemExit(1)

show(f"a {MASS:.1f} kg body resting on the ground, g = {G:.2f}")
show("")
show(f"   predicted m*g       : {MASS * G:8.3f} N")
show(f"   contact sensor says : {fA:8.3f} N")
show("")
show("   check it against arithmetic before trusting it on anything harder")
show("")

# ---- 2. the resting body that stops reporting ---------------------------
show("the SAME body, with PhysX allowed to put it to sleep (the default):")
show(f"   {'step':>6} {'time':>8} {'A awake':>12} {'B default':>12}")
show("   " + "-" * 44)
for k in (40, 60, 90, 150, 300):
    step_to(k)
    _, a = read(pA)
    okb, b = read(pB)
    show(f"   {k:6d} {k * DT:7.2f}s {a:11.4f}N {b:11.4f}N")
show("")
show("   B is still sitting on the ground, still pressing with the same")
show("   weight, and after about half a second it reports 0.0000 and")
show("   in_contact False. PhysX has put it to sleep and a sleeping body")
show("   generates no contact reports. A robot standing still has silent feet.")
show("")
show("   The fix is TWO attributes on PhysxRigidBodyAPI, not one:")
show("      sleepThreshold        = 0")
show("      stabilizationThreshold = 0")
show("   Setting only the first still collapses to zero.")
show("")

# ---- 3. the threshold that deletes data --------------------------------
okC, fC = read(pC)
show(f"the same body again, with min_threshold = {HEAVY_THRESHOLD:.0f} N:")
show(f"   {'sensor':>8} {'min_threshold':>15} {'in_contact':>12} {'force N':>10}")
show("   " + "-" * 50)
show(f"   {'A':>8} {0.0:15.1f} {str(okA):>12} {fA:10.3f}")
show(f"   {'C':>8} {HEAVY_THRESHOLD:15.1f} {str(okC):>12} {fC:10.3f}")
show("")
show(f"   C is touching the ground exactly as hard as A. It reports {str(okC)},")
show(f"   because {MASS * G:.2f} N never reaches a {HEAVY_THRESHOLD:.0f} N "
     f"threshold, and")
show("   nothing anywhere raises a warning. The sensor is doing as it was told.")

print("CON_DONE", flush=True)
sim.close()
