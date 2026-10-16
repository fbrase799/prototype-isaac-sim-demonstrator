# 4.1: read the APPLIED SCHEMAS and the physics defaults out of a running
# stage, because the lecture quotes both and neither should come from the docs.
#
# Two things this proves on camera:
#   1. The applied schema list says PhysicsRigidBodyAPI and PhysicsCollisionAPI.
#      The Python classes are UsdPhysics.RigidBodyAPI and .CollisionAPI, so the
#      string you match against is NOT the name you typed.
#   2. A fresh rigid body reports mass 0.0, diagonalInertia (0,0,0) and
#      centerOfMass (-inf,-inf,-inf). Those are sentinels meaning "compute this
#      from the collider at simulation start", not a broken body, and they are
#      the setup for 4.3 and 4.5.
#
# SimulationApp is constructed FIRST, before any pxr import: python.sh does not
# put omni.usd.libs on PYTHONPATH, so `from pxr import ...` fails until the Kit
# runtime that owns the USD libraries has booted. Same rule as 3.8 and 3.9.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics, Gf

stage = Usd.Stage.CreateInMemory()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

CASES = [
    ("both",      True,  True),
    ("body_only", True,  False),
    ("coll_only", False, True),
    ("neither",   False, False),
]

for name, rigid, coll in CASES:
    c = UsdGeom.Cube.Define(stage, f"/World/{name}")
    c.CreateSizeAttr(0.5)
    if rigid:
        UsdPhysics.RigidBodyAPI.Apply(c.GetPrim())
    if coll:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())

ground = UsdGeom.Cube.Define(stage, "/World/ground")
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

print("")
print("PHYSX_SCHEMAS")
print(f"{'prim':<20} {'applied schemas'}")
print("-" * 68)
for name, _r, _c in CASES + [("ground", False, True)]:
    prim = stage.GetPrimAtPath(f"/World/{name}")
    applied = list(prim.GetAppliedSchemas())
    print(f"/World/{name:<13} {applied if applied else '[]  (nothing applied)'}")

# The defaults, off the prim that has a body, exactly as the property panel
# shows them.
print("")
print("PHYSX_DEFAULTS  (a fresh rigid body, nothing set by hand)")
print("-" * 68)
prim = stage.GetPrimAtPath("/World/both")
UsdPhysics.MassAPI.Apply(prim)
for attr in ("physics:mass", "physics:density", "physics:diagonalInertia",
             "physics:centerOfMass", "physics:rigidBodyEnabled",
             "physics:collisionEnabled"):
    a = prim.GetAttribute(attr)
    print(f"  {attr:<28} = {a.Get() if a else 'not present'}")
print("")
print("  mass 0.0 and inertia (0,0,0) are SENTINELS: compute me from the")
print("  collider. -inf centre of mass is the same message.")
print("")
print("PHYSX_SCHEMAS_DONE", flush=True)

sim.close()
