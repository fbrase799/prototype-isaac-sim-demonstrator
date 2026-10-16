# 4.4: the inertia tensor, derived and then CHECKED against the simulator.
#
# The lecture works the box and cylinder formulae by hand. This script is what
# makes that honest: it builds the same shapes in Isaac, reads the tensor PhysX
# computed, and prints the hand formula beside it so the two can be compared on
# camera. If the derivation is right the columns match; if it is wrong, the
# lecture would be teaching a formula the engine disagrees with.
#
# The formulae being checked, for a body of mass m about its centre of mass:
#
#   solid box, sides a b c:   Ixx = m(b² + c²)/12   and cyclic
#   solid cylinder, r, h:     Izz = m r²/2 ,  Ixx = Iyy = m(3r² + h²)/12
#
# 4.3 already measured a 0.4 m cube at 1.7066670656204224, and m s²/6 for
# m = 64 gives 1.70666..., which is the box formula with b = c = s. So this is
# the same check, generalised to a non-cubic box and a cylinder.
#
# SimulationApp FIRST, before any pxr import.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})

import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf
import omni.usd
from isaacsim.core.utils.stage import clear_stage
from isaacsim.core.api import World

WORLD = World(stage_units_in_meters=1.0)
DENSITY = 1000.0


def fresh():
    clear_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    w = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(w.GetPrim())
    sc = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    sc.CreateGravityMagnitudeAttr(9.81)
    return stage


def read_back(path, steps=8):
    WORLD.reset()
    for _ in range(steps):
        WORLD.step(render=False)
    from isaacsim.core.prims import RigidPrim
    rp = RigidPrim(path)
    rp.initialize()
    mass = float(np.asarray(rp.get_masses())[0])
    it = np.asarray(rp.get_inertias())[0]
    return mass, (float(it[0]), float(it[4]), float(it[8]))


print("")
print("PHYSX_INERTIA")
print("a solid BOX, sides a x b x c, density 1000")
print(f"{'sides (m)':<20} {'mass':>9} {'Ixx sim':>11} {'Ixx hand':>11} {'Iyy sim':>11} {'Izz sim':>11}")
print("-" * 80)

BOXES = [(0.4, 0.4, 0.4), (0.8, 0.4, 0.2), (1.2, 0.3, 0.6)]
for (a, b, c) in BOXES:
    stage = fresh()
    # A Cube prim is uniform, so a non-cubic box is a scaled cube: scale is the
    # only way to get distinct side lengths out of UsdGeom.Cube.
    cube = UsdGeom.Cube.Define(stage, "/World/box")
    cube.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(cube)
    x.AddTranslateOp().Set(Gf.Vec3d(0, 0, 2.0))
    x.AddScaleOp().Set(Gf.Vec3f(a, b, c))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateDensityAttr(DENSITY)
    try:
        m, (ixx, iyy, izz) = read_back("/World/box")
        hand = m * (b * b + c * c) / 12.0
        print(f"{a} x {b} x {c:<12} {m:9.3f} {ixx:11.5f} {hand:11.5f} {iyy:11.5f} {izz:11.5f}")
    except Exception as exc:                      # noqa: BLE001
        print(f"{a} x {b} x {c:<12} FAILED {type(exc).__name__}: {str(exc)[:24]}")

print("")
print("a solid CYLINDER, radius r, height h, density 1000")
print(f"{'r, h (m)':<20} {'mass':>9} {'Izz sim':>11} {'Izz hand':>11} {'Ixx sim':>11} {'Ixx hand':>11}")
print("-" * 80)

CYLS = [(0.25, 0.60), (0.40, 0.20)]
for (r, h) in CYLS:
    stage = fresh()
    cyl = UsdGeom.Cylinder.Define(stage, "/World/cyl")
    cyl.CreateRadiusAttr(r)
    cyl.CreateHeightAttr(h)
    cyl.CreateAxisAttr("Z")
    UsdGeom.Xformable(cyl).AddTranslateOp().Set(Gf.Vec3d(0, 0, 2.0))
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(cyl.GetPrim())
    UsdPhysics.MassAPI.Apply(cyl.GetPrim()).CreateDensityAttr(DENSITY)
    try:
        m, (ixx, iyy, izz) = read_back("/World/cyl")
        izz_hand = m * r * r / 2.0
        ixx_hand = m * (3 * r * r + h * h) / 12.0
        print(f"r={r}, h={h:<11} {m:9.3f} {izz:11.5f} {izz_hand:11.5f} {ixx:11.5f} {ixx_hand:11.5f}")
    except Exception as exc:                      # noqa: BLE001
        print(f"r={r}, h={h:<11} FAILED {type(exc).__name__}: {str(exc)[:24]}")

print("")
print("PHYSX_INERTIA_DONE", flush=True)
sim.close()
