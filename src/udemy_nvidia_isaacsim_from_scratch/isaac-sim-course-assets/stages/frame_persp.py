import omni.usd
from pxr import UsdGeom, Gf
stage = omni.usd.get_context().get_stage()
for path in ("/OmniverseKit_Persp", "/World/shotCam"):
    p = stage.GetPrimAtPath(path)
    if not p: continue
    x = UsdGeom.Xformable(p)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(1.30, -1.70, 0.68))
    x.AddRotateXYZOp().Set(Gf.Vec3f(79, 0, 38))
    c = UsdGeom.Camera(p)
    if c: c.GetFocalLengthAttr().Set(40.0)
    print("[frame]", path, "moved")
