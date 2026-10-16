# 3.7: cameras, framing a shot deliberately, and why the viewport camera is not
# the camera you author.
#
# This lecture has a real failure at its centre and I hit it for real while
# building section 3: writing /OmniverseKit_Persp's xformOps does not stick.
# The stage below demonstrates BOTH cameras so the difference is on screen:
#
#   /OmniverseKit_Persp   Kit's own perspective camera. Owned by the viewport's
#                         manipulator, which writes its state back. Your edits
#                         are overwritten and nothing errors.
#   /World/shot_cam       a camera prim YOU authored. Nothing else owns it, so
#                         what you write is what it is.
#
# CAM_MODE picks which the viewport looks through, so a take can show the same
# scene from an authored camera and from the default one.
import os

import omni.usd
from pxr import Usd, UsdGeom, UsdLux, Gf, Sdf

MODE = os.environ.get("CAM_MODE", "authored")   # authored | default

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

# Kill Kit's default distant light so the shot is lit by what we author (3.6).
for prim in stage.Traverse():
    if prim.IsA(UsdLux.DistantLight) and not prim.GetPath().pathString.startswith("/World"):
        prim.SetActive(False)

ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(ground)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
gx.AddScaleOp().Set(Gf.Vec3f(30, 30, 0.1))
ground.CreateDisplayColorAttr([(0.70, 0.72, 0.74)])

# A subject worth framing: a short row of blocks at known positions, so "the
# shot is composed" is checkable rather than a matter of taste.
SUBJECT = [(-0.9, 0.0, 0.25, 0.5), (0.0, 0.15, 0.35, 0.7), (0.95, -0.1, 0.22, 0.44)]
for i, (x, y, h, s) in enumerate(SUBJECT):
    c = UsdGeom.Cube.Define(stage, f"/World/block_{i}")
    c.CreateSizeAttr(1.0)
    cx = UsdGeom.Xformable(c)
    cx.AddTranslateOp().Set(Gf.Vec3d(x, y, h))
    cx.AddScaleOp().Set(Gf.Vec3f(s, s, h * 2))
    c.CreateDisplayColorAttr([(0.55, 0.38, 0.18) if i == 1 else (0.80, 0.81, 0.82)])

dome = UsdLux.DomeLight.Define(stage, "/World/dome")
dome.CreateIntensityAttr(1100.0)
dome.CreateColorAttr(Gf.Vec3f(0.95, 0.96, 1.0))

# THE CAMERA YOU AUTHOR. A real prim, in the stage, that you own.
#
# focalLength and the aperture are the two numbers that actually decide the
# field of view, and they are attributes like any other: this is the whole
# reason an authored camera is repeatable and a dragged viewport is not.
FOCAL = float(os.environ.get("CAM_FOCAL", "35"))
shot = UsdGeom.Camera.Define(stage, "/World/shot_cam")
shot.CreateFocalLengthAttr(FOCAL)
shot.CreateHorizontalApertureAttr(20.955)      # 35 mm full-frame, in mm
shot.CreateVerticalApertureAttr(15.2908)
shot.CreateClippingRangeAttr(Gf.Vec2f(0.05, 10000.0))
shot.CreateFocusDistanceAttr(3.2)
shot.CreateFStopAttr(0.0)                      # 0 = no depth of field

EYE = Gf.Vec3d(2.1, -3.4, 1.55)
TARGET = Gf.Vec3d(0.0, 0.05, 0.42)
m = Gf.Matrix4d().SetLookAt(EYE, TARGET, Gf.Vec3d(0, 0, 1)).GetInverse()
UsdGeom.Xformable(shot.GetPrim()).AddTransformOp().Set(m)


def fov_degrees(focal_mm, aperture_mm):
    import math
    return 2.0 * math.degrees(math.atan(aperture_mm / (2.0 * focal_mm)))


print(f"FACT focal {FOCAL} aperture 20.955 -> horiz fov "
      f"{fov_degrees(FOCAL, 20.955):.1f} deg", flush=True)
print(f"FACT eye {tuple(round(v,2) for v in EYE)} "
      f"target {tuple(round(v,2) for v in TARGET)}", flush=True)

import asyncio
import omni.kit.app


async def _frame():
    app = omni.kit.app.get_app()
    for _ in range(700):
        await app.next_update_async()
    try:
        import omni.kit.viewport.utility as vpu
        vp = vpu.get_active_viewport()
        if MODE == "authored":
            vp.camera_path = "/World/shot_cam"
        else:
            # Leave the viewport on Kit's own camera, and try to place it the
            # way the docs suggest. THIS IS THE FAILURE and it is silent: the
            # write is accepted and then overwritten by the viewport's own
            # manipulator, which owns that prim.
            vp.camera_path = "/OmniverseKit_Persp"
            k = stage.GetPrimAtPath("/OmniverseKit_Persp")
            before = k.GetAttribute("xformOp:translate").Get()
            k.GetAttribute("xformOp:translate").Set(Gf.Vec3d(2.1, -3.4, 1.55))
            for _ in range(60):
                await app.next_update_async()
            after = k.GetAttribute("xformOp:translate").Get()
            print(f"FACT kit_cam_before {before}", flush=True)
            print(f"FACT kit_cam_after_write {after}", flush=True)
            print(f"FACT kit_cam_write_stuck {after == Gf.Vec3d(2.1, -3.4, 1.55)}",
                  flush=True)
        print(f"FACT viewport_camera {vp.camera_path} mode={MODE}", flush=True)
        print("FACT cam_set", flush=True)
    except Exception as e:
        print("FRAME_FAILED", e, flush=True)


asyncio.ensure_future(_frame())
print("USD_CAMERAS_READY", flush=True)
