# 4.9: reading physics back out of a running stage.
#
# G5 in practice, and the tool the whole section has quietly depended on. Every
# number in lectures 4.1 to 4.8 came from this: build a stage, step it, and ask
# the simulator what it actually has rather than what you asked for.
#
# The lecture demonstrates the three things you will actually want:
#   1. What is IN the stage: every rigid body, its schemas, mass and inertia.
#   2. What is HAPPENING: live pose and velocity while it runs.
#   3. What CONTACTS exist, which is how you answer "is it touching the ground".
#
# SimulationApp FIRST, before any pxr import.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf
import omni.usd
from isaacsim.core.utils.stage import clear_stage
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim

WORLD = World(stage_units_in_meters=1.0)


def show(line=""):
    print("READ> " + line, flush=True)


clear_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())
sc = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
sc.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
sc.CreateGravityMagnitudeAttr(9.81)

ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(ground)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
gx.AddScaleOp().Set(Gf.Vec3f(10, 10, 0.1))
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

# a small scene with three different bodies, so the readback has variety
SPECS = [
    ("crate", (0.4, 0.4, 0.4), 1.0, 12.0),
    ("panel", (0.9, 0.1, 0.6), 1.4, 4.0),
    ("block", (0.3, 0.3, 0.9), 1.8, 20.0),
]
for name, size, z, mass in SPECS:
    c = UsdGeom.Cube.Define(stage, f"/World/{name}")
    c.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(c)
    x.AddTranslateOp().Set(Gf.Vec3d(0, 0, z))
    x.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(c.GetPrim())
    UsdPhysics.MassAPI.Apply(c.GetPrim()).CreateMassAttr(mass)

WORLD.reset()

# ---------------------------------------------------------------------------
# 1. WHAT IS IN THE STAGE. Traverse and report, without knowing the names.
# ---------------------------------------------------------------------------
show("1. what is in the stage: found by traversing, not by knowing the names")
show(f"   {'prim':>16} {'mass':>8} {'Ixx':>9} {'Iyy':>9} {'Izz':>9}")
show("   " + "-" * 56)
found = []
for prim in stage.Traverse():
    if "PhysicsRigidBodyAPI" not in prim.GetAppliedSchemas():
        continue
    rp = RigidPrim(str(prim.GetPath()))
    rp.initialize()
    m = float(np.asarray(rp.get_masses())[0])
    it = np.asarray(rp.get_inertias())[0]
    show(f"   {prim.GetName():>16} {m:8.2f} {it[0]:9.4f} {it[4]:9.4f} {it[8]:9.4f}")
    found.append((prim.GetName(), rp))
show(f"   found {len(found)} rigid bodies without naming any of them")
show("")

# ---------------------------------------------------------------------------
# 2. WHAT IS HAPPENING: pose and velocity while it runs.
# ---------------------------------------------------------------------------
show("2. what is happening: the crate's height and speed as it falls")
show(f"   {'step':>6} {'z (m)':>9} {'vz (m/s)':>10}")
show("   " + "-" * 30)
crate = dict(found)["crate"]
for step in range(0, 61):
    WORLD.step(render=False)
    if step % 12 == 0:
        z = float(np.asarray(crate.get_world_poses()[0])[0][2])
        vz = float(np.asarray(crate.get_linear_velocities())[0][2])
        show(f"   {step:6d} {z:9.4f} {vz:10.4f}")
show("")

# ---------------------------------------------------------------------------
# 3. IS IT RESTING? Answered from velocity, because contact forces would not
#    come out of this build.
#
#    get_net_contact_forces() returns nan unless the RigidPrim was constructed
#    with track_contact_forces=True, and it only WARNS about that in the log.
#    Constructing it that way during the stage traversal raises inside
#    RigidContactView, and constructing it before World.reset() raises "The
#    Physics Context's physics scene path is invalid" (the same constraint that
#    blocked the timestep test in 4.7). Rather than teach an API dance I could
#    not make work, the lecture answers the actual question a different way:
#    a body at rest has velocity at zero and is not moving between steps.
# ---------------------------------------------------------------------------
for _ in range(120):
    WORLD.step(render=False)

show("3. is it resting? asked of the velocity, which always reads")
show(f"   {'prim':>16} {'z (m)':>9} {'|v| (m/s)':>11} {'|w| (rad/s)':>13}   verdict")
show("   " + "-" * 62)
for name, rp in found:
    z = float(np.asarray(rp.get_world_poses()[0])[0][2])
    v = float(np.linalg.norm(np.asarray(rp.get_linear_velocities())[0]))
    w_ = float(np.linalg.norm(np.asarray(rp.get_angular_velocities())[0]))
    verdict = "at rest" if (v < 0.01 and w_ < 0.01) else "still moving"
    show(f"   {name:>16} {z:9.4f} {v:11.5f} {w_:13.5f}   {verdict}")
show("")
show("   NOTE: get_net_contact_forces() needs track_contact_forces=True at")
show("   construction, and returns nan without it while only warning in the log.")

show("")
print("PHYSX_READBACK_DONE", flush=True)
sim.close()
