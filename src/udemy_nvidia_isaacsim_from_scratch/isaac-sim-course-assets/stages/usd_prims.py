# 3.1: prims, paths and the stage tree, built so the tree is worth looking at.
# Deliberately mixes prim TYPES (Xform, Cube, Sphere, DomeLight, Scope) so the
# lecture's claim -- "the Stage lists everything that exists, not everything
# visible" -- is visible in the tree itself.
import omni.usd
from pxr import UsdGeom, UsdLux, UsdPhysics, Gf

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

# a Scope: exists, groups things, and is INVISIBLE. The point of the lecture.
UsdGeom.Scope.Define(stage, "/World/props")

g = UsdGeom.Cube.Define(stage, "/World/ground"); g.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(g); gx.AddTranslateOp().Set(Gf.Vec3d(0,0,-0.05))
gx.AddScaleOp().Set(Gf.Vec3f(20,20,0.1))
g.CreateDisplayColorAttr([(0.62,0.64,0.66)])

for i,(n,c) in enumerate([("box_a",(0.35,0.62,0.05)),
                          ("box_b",(0.75,0.76,0.78)),
                          ("box_c",(0.35,0.62,0.05))]):
    b = UsdGeom.Cube.Define(stage, f"/World/props/{n}")
    b.CreateSizeAttr(0.4)
    bx = UsdGeom.Xformable(b)
    bx.AddTranslateOp().Set(Gf.Vec3d(-0.8 + i*0.8, 0, 0.2))
    b.CreateDisplayColorAttr([c])

s = UsdGeom.Sphere.Define(stage, "/World/props/ball")
s.CreateRadiusAttr(0.22)
UsdGeom.Xformable(s).AddTranslateOp().Set(Gf.Vec3d(0, 1.0, 0.22))
s.CreateDisplayColorAttr([(0.85,0.86,0.88)])

UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(1100.0)
print("USD_PRIMS_STAGE_READY", flush=True)
