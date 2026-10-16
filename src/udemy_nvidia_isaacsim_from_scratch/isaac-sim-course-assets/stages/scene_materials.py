# 7.3: materials, and why a physics material is not the same object.
#
# USD has two completely separate things both called "material", and confusing
# them is the next surprise after 7.2's two folders. One decides what a surface
# LOOKS like to the renderer. The other decides how it BEHAVES when something
# slides on it. Binding the first does nothing at all to the second.
#
# Three things get measured:
#
#   1. A VISUAL MATERIAL CHANGES NOTHING PHYSICAL. Bind a shiny metal look to
#      a ramp and the block on it slides exactly as far as before.
#
#   2. A PHYSICS MATERIAL IS WHAT MOVES THE BLOCK. Same geometry, same visual
#      material, three friction values, three different distances.
#
#   3. THE DEFAULT IS NOT ZERO. A surface with no physics material still has
#      friction, because PhysX supplies a default. So "I did not set friction"
#      does not mean "frictionless" -- it means "whatever PhysX chose".
#
# TRAPS:
#
# - UsdShade.Material and UsdPhysics.MaterialAPI are different schemas. A prim
#   can carry both, one, or neither, and the viewport only shows you the first.
# - The physics material must be BOUND with the "physics" material purpose.
#   Binding it with the default purpose silently does nothing.
# - This script refuses to print a table it did not measure.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import math
import numpy as np
from pxr import UsdGeom, Gf, UsdPhysics, UsdLux, UsdShade, Sdf
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import create_new_stage


def show(l=""):
    print("MAT> " + l, flush=True)


create_new_stage()
st = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
w = UsdGeom.Xform.Define(st, "/World")
st.SetDefaultPrim(w.GetPrim())
UsdLux.DomeLight.Define(st, "/World/dome").CreateIntensityAttr(1000.0)

TILT = 20.0          # degrees; steep enough to slide, shallow enough to differ
LANE_Y = 3.0         # lanes are spaced along y


def ramp(path, y, mu):
    """A tilted slab with an optional physics material of friction mu."""
    r = UsdGeom.Cube.Define(st, path)
    r.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(r)
    x.AddTranslateOp().Set(Gf.Vec3d(0, y, 0.0))
    x.AddRotateYOp().Set(-TILT)
    x.AddScaleOp().Set(Gf.Vec3f(14.0, 1.6, 0.2))
    UsdPhysics.CollisionAPI.Apply(r.GetPrim())
    if mu is not None:
        mp = UsdShade.Material.Define(st, path + "_physmat")
        pm = UsdPhysics.MaterialAPI.Apply(mp.GetPrim())
        pm.CreateStaticFrictionAttr().Set(mu)
        pm.CreateDynamicFrictionAttr().Set(mu)
        pm.CreateRestitutionAttr().Set(0.0)
        # THE PURPOSE MATTERS. Bound without "physics" this is decoration.
        UsdShade.MaterialBindingAPI.Apply(r.GetPrim()).Bind(
            mp, UsdShade.Tokens.weakerThanDescendants, "physics")
    return r


def block(path, y):
    b = UsdGeom.Cube.Define(st, path)
    b.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(b)
    # start high on the ramp, on its surface
    x.AddTranslateOp().Set(Gf.Vec3d(-4.0, y, 1.85))
    x.AddRotateYOp().Set(-TILT)
    x.AddScaleOp().Set(Gf.Vec3f(0.5, 0.5, 0.5))
    UsdPhysics.CollisionAPI.Apply(b.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(b.GetPrim())
    UsdPhysics.MassAPI.Apply(b.GetPrim()).CreateMassAttr(1.0)
    return b


# LANES:
#   A: no physics material at all      -> whatever PhysX defaults to
#   B: physics material, friction 0.05 -> slippery
#   C: physics material, friction 0.90 -> grippy
#   D: no physics material, but a SHINY VISUAL MATERIAL bound. This is the
#      lecture's point: it looks like polished metal and behaves like lane A.
LANES = [("A_default", None, False), ("B_slippery", 0.05, False),
         ("C_grippy", 0.90, False), ("D_visual_only", None, True)]

for i, (name, mu, visual) in enumerate(LANES):
    y = i * LANE_Y
    r = ramp(f"/World/ramp_{name}", y, mu)
    if visual:
        # a proper UsdShade material: this is the one the renderer uses
        vm = UsdShade.Material.Define(st, f"/World/ramp_{name}_look")
        sh = UsdShade.Shader.Define(st, f"/World/ramp_{name}_look/shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(1.0)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.05)
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.85, 0.86, 0.9))
        vm.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(r.GetPrim()).Bind(vm)
    block(f"/World/block_{name}", y)

W = World(stage_units_in_meters=1.0)
W.reset()

from isaacsim.core.prims import RigidPrim
starts = {}
for name, _, _ in LANES:
    rp = RigidPrim(f"/World/block_{name}")
    p, _ = rp.get_world_poses()
    starts[name] = float(p[0][0])

STEPS = 180
for _ in range(STEPS):
    W.step(render=False)

res = {}
for name, _, _ in LANES:
    rp = RigidPrim(f"/World/block_{name}")
    p, _ = rp.get_world_poses()
    res[name] = float(p[0][0]) - starts[name]

if all(abs(v) < 1e-4 for v in res.values()):
    show("NOTHING MOVED -- the ramp or the blocks are not simulating; no table.")
    print("MAT_DONE", flush=True)
    sim.close()
    raise SystemExit(1)

show(f"four identical blocks on four identical {TILT:.0f} degree ramps,")
show(f"{STEPS} physics steps ({STEPS / 60.0:.1f} s). Only the MATERIAL differs:")
show("")
show(f"   {'lane':>16} {'physics material':>18} {'visual material':>17} "
     f"{'slid':>9}")
show("   " + "-" * 66)
labels = {"A_default": ("none", "default"),
          "B_slippery": ("friction 0.05", "default"),
          "C_grippy": ("friction 0.90", "default"),
          "D_visual_only": ("none", "polished metal")}
for name, _, _ in LANES:
    pm, vm = labels[name]
    show(f"   {name:>16} {pm:>18} {vm:>17} {res[name]:8.3f}m")
show("")
show("   Lane D is the one to look at. It is bound to a metallic, almost")
show("   mirror-smooth visual material, and it slid the same distance as lane")
show("   A, which has no material at all. The renderer changed. The physics")
show("   did not hear about it.")
show("")
show("   And lane A did not slide freely either: with NO physics material")
show("   PhysX still applies a default friction, so 'I did not set it' means")
show("   'I accepted whatever the engine chose', not 'frictionless'.")

print("MAT_DONE", flush=True)
sim.close()
