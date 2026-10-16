# Frame the shot camera on the robot now that the reference has resolved.
import omni.usd
from pxr import UsdGeom, Gf
stage = omni.usd.get_context().get_stage()
cam = stage.GetPrimAtPath("/World/shotCam")
if cam:
    x = UsdGeom.Xformable(cam)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(1.35, -1.75, 0.72))
    x.AddRotateXYZOp().Set(Gf.Vec3f(80, 0, 38))
    UsdGeom.Camera(cam).GetFocalLengthAttr().Set(38.0)
try:
    import omni.kit.viewport.utility as vpu
    vp = vpu.get_active_viewport()
    vp.camera_path = "/World/shotCam"
    print("[frame] shotCam active")
except Exception as e:
    print("[frame] err", e)
