# 3.2: the transform stack, and why ORDER matters.
#
# Two identical cubes with the same two operations in opposite orders. USD applies
# xformOps in the order they appear in xformOpOrder, so translate-then-rotate and
# rotate-then-translate end up in visibly different places. That is the lecture.
import omni.usd
from pxr import UsdGeom, UsdLux, Gf

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
w = UsdGeom.Xform.Define(stage, "/World"); stage.SetDefaultPrim(w.GetPrim())

g = UsdGeom.Cube.Define(stage, "/World/ground"); g.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(g); gx.AddTranslateOp().Set(Gf.Vec3d(0,0,-0.05))
gx.AddScaleOp().Set(Gf.Vec3f(24,24,0.1))
g.CreateDisplayColorAttr([(0.62,0.64,0.66)])

# A: translate 3m out along X, THEN rotate 50 deg about Z.
a = UsdGeom.Cube.Define(stage, "/World/translate_then_rotate")
a.CreateSizeAttr(0.5); a.CreateDisplayColorAttr([(0.46,0.72,0.05)])
ax = UsdGeom.Xformable(a)
ax.AddTranslateOp().Set(Gf.Vec3d(3.0, 0, 0.25))
ax.AddRotateZOp().Set(50.0)

# B: rotate 50 deg about Z FIRST, then translate the same 3m.
b = UsdGeom.Cube.Define(stage, "/World/rotate_then_translate")
b.CreateSizeAttr(0.5); b.CreateDisplayColorAttr([(0.85,0.30,0.32)])
bx = UsdGeom.Xformable(b)
bx.AddRotateZOp().Set(50.0)
bx.AddTranslateOp().Set(Gf.Vec3d(3.0, 0, 0.25))

# a marker at the origin they both rotate about
o = UsdGeom.Sphere.Define(stage, "/World/origin_marker")
o.CreateRadiusAttr(0.12); o.CreateDisplayColorAttr([(0.30,0.32,0.34)])
UsdGeom.Xformable(o).AddTranslateOp().Set(Gf.Vec3d(0,0,0.12))

UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(1100.0)

for p in ("/World/translate_then_rotate", "/World/rotate_then_translate"):
    pr = stage.GetPrimAtPath(p)
    print("ORDER", p, list(UsdGeom.Xformable(pr).GetXformOpOrderAttr().Get()), flush=True)
# Frame both cubes and the origin marker. Without this the default camera cuts
# the green one off, and the whole point is SEEING the two land differently.
import asyncio, omni.kit.app
async def _frame():
    app = omni.kit.app.get_app()
    for _ in range(320):
        await app.next_update_async()
    try:
        import omni.kit.viewport.utility as vpu
        cam = stage.GetPrimAtPath("/OmniverseKit_Persp")
        if cam:
            cam.GetAttribute("xformOp:translate").Set(Gf.Vec3d(6.5, -7.5, 4.6))
            r = cam.GetAttribute("xformOp:rotateYXZ") or cam.GetAttribute("xformOp:rotateXYZ")
            if r: r.Set(Gf.Vec3f(66, 0, 41))
    except Exception as e:
        print("FRAME_FAILED", e, flush=True)
asyncio.ensure_future(_frame())
print("USD_XFORMS_READY", flush=True)
