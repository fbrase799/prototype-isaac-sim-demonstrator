# 3.4 FAILURE: a Y-up centimetres asset referenced onto a Z-up metres stage.
#
# The asset arrives lying on its side and a hundred times too big, and nothing
# errors. That is the whole lecture, so the stage has to produce the wrong
# result FOR REAL rather than draw a picture of it.
#
# Three prims, one component:
#   /World/broken   the vendor file referenced raw. Sideways and 100x.
#   /World/fixed    the same file, corrected the way you actually correct it.
#   /World/correct  a native prim at the intended size, as the ruler.
#
# The correction is deliberately the WRAPPER approach rather than editing the
# vendor file: you do not own that file, 3.3 just established that editing it
# changes it everywhere, and it may be replaced by the vendor tomorrow.
import omni.usd
from pxr import Usd, UsdGeom, UsdLux, Sdf, Gf

# Authored in the vendor's convention, not ours. Written to out/ because
# stages/ is a host dir owned by uid 1000 and this container runs as 1234; a
# failed write there is silent and leaves the reference resolving to nothing.
VENDOR = "/workspace/out/vendor_crate_ycm.usda"

# What the crate is MEANT to be, in metres: 0.6 x 0.6 x 0.45.
TRUE_W, TRUE_D, TRUE_H = 0.6, 0.6, 0.45


def build_vendor_asset():
    """Author the asset the way a Y-up centimetres pipeline would.

    Two independent mismatches, and they are worth keeping separate because
    they have different fixes:
      metersPerUnit = 0.01   the numbers mean centimetres, so 60 means 0.6 m
      upAxis        = "Y"    the asset's up is our Y, so it lands on its side
    """
    layer = Sdf.Layer.CreateNew(VENDOR)
    sub = Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(sub, UsdGeom.Tokens.y)        # THEIR up
    UsdGeom.SetStageMetersPerUnit(sub, 0.01)             # THEIR unit
    crate = UsdGeom.Xform.Define(sub, "/crate")
    sub.SetDefaultPrim(crate.GetPrim())
    # 60 x 45 x 60 of THEIR units = 0.6 x 0.45 x 0.6 m, with height on Y.
    body = UsdGeom.Cube.Define(sub, "/crate/body")
    body.CreateSizeAttr(1.0)
    UsdGeom.Xformable(body).AddScaleOp().Set(Gf.Vec3f(60.0, 45.0, 60.0))
    body.CreateDisplayColorAttr([(0.55, 0.38, 0.18)])
    # A lid, so "which way is up" is visible rather than inferred: on a cube
    # alone, lying on its side looks identical to standing up.
    lid = UsdGeom.Cube.Define(sub, "/crate/lid")
    lid.CreateSizeAttr(1.0)
    lx = UsdGeom.Xformable(lid)
    lx.AddTranslateOp().Set(Gf.Vec3d(0.0, 24.0, 0.0))    # up is +Y for them
    lx.AddScaleOp().Set(Gf.Vec3f(64.0, 6.0, 64.0))
    lid.CreateDisplayColorAttr([(0.30, 0.22, 0.11)])
    layer.Save()
    return layer


build_vendor_asset()
if not Sdf.Layer.FindOrOpen(VENDOR):
    raise RuntimeError(f"vendor asset {VENDOR} was not written; the reference "
                       "would resolve to nothing and the lecture would show "
                       "an empty stage instead of a wrong one")

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)          # OUR up
UsdGeom.SetStageMetersPerUnit(stage, 1.0)                # OUR unit
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(ground)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
gx.AddScaleOp().Set(Gf.Vec3f(220, 220, 0.1))    # the broken crate is 64 m wide
ground.CreateDisplayColorAttr([(0.62, 0.64, 0.66)])


def set_translate(prim, v):
    """3.2's rule. A referenced prim may already carry a translate op."""
    x = UsdGeom.Xformable(prim)
    for op in x.GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            op.Set(v)
            return
    x.AddTranslateOp().Set(v)


# 1. THE BUG. Reference it and change nothing.
broken = UsdGeom.Xform.Define(stage, "/World/broken")
broken.GetPrim().GetReferences().AddReference(VENDOR)
# Pushed well back in +Y. At y=22 the 64 m crate filled the BACKGROUND of the
# tight shot as a featureless wall, so the fixed/correct comparison had no
# readable scene behind it. At y=90 it is still comfortably inside the wide
# shot and completely out of the tight one.
set_translate(broken.GetPrim(), Gf.Vec3d(0.0, 90.0, 0.0))

# 2. THE FIX, on a wrapper we own, not on the vendor's file.
#    scale 0.01 converts their centimetres to our metres.
#    rotateX +90 maps their +Y up onto our +Z up.
#    Order matters (3.2): scale, then rotate, then translate.
fixed = UsdGeom.Xform.Define(stage, "/World/fixed")
fixed.GetPrim().GetReferences().AddReference(VENDOR)
fx = UsdGeom.Xformable(fixed)
fx.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))
fx.AddRotateXOp().Set(90.0)
set_translate(fixed.GetPrim(), Gf.Vec3d(-1.1, 0.0, 0.225))

# 3. A native crate at the true size, so "correct" is a thing on screen and not
#    an assertion in the voiceover.
ref = UsdGeom.Cube.Define(stage, "/World/correct")
ref.CreateSizeAttr(1.0)
rx = UsdGeom.Xformable(ref)
rx.AddTranslateOp().Set(Gf.Vec3d(1.1, 0.0, TRUE_H / 2))
rx.AddScaleOp().Set(Gf.Vec3f(TRUE_W, TRUE_D, TRUE_H))
ref.CreateDisplayColorAttr([(0.36, 0.50, 0.22)])

UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(1100.0)


def extent_of(path):
    """World-space bounding box, so the size claims are MEASURED."""
    p = stage.GetPrimAtPath(path)
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_]).ComputeWorldBound(p)
    r = bb.ComputeAlignedRange()
    return r.GetSize()


for path in ("/World/broken", "/World/fixed", "/World/correct"):
    s = extent_of(path)
    print(f"FACT extent {path} x={s[0]:.3f} y={s[1]:.3f} z={s[2]:.3f}", flush=True)
print("FACT stage_up", UsdGeom.GetStageUpAxis(stage),
      "mpu", UsdGeom.GetStageMetersPerUnit(stage), flush=True)
vs = Usd.Stage.Open(VENDOR)
print("FACT vendor_up", UsdGeom.GetStageUpAxis(vs),
      "mpu", UsdGeom.GetStageMetersPerUnit(vs), flush=True)

import asyncio
import omni.kit.app


async def _frame():
    """Place the viewport camera once Kit has finished its own startup framing.

    Setting the existing xformOp ATTRIBUTES does not work, and it fails without
    an error: probed afterwards the camera was still at Kit's default (5, 5, 5)
    with 400 ticks AND with 700. What sticks is carter_walkthrough's route,
    which is why it is copied here rather than improved on:
      - resolve the camera through the ACTIVE VIEWPORT, not the hardcoded
        /OmniverseKit_Persp path
      - ClearXformOpOrder() and re-add the ops, instead of Set()ing them
    Waiting alone is not the fix; clearing the order is.
    """
    app = omni.kit.app.get_app()
    for _ in range(700):
        await app.next_update_async()
    try:
        import omni.kit.viewport.utility as vpu
        # OWN CAMERA, then point the viewport at it. Writing to
        # /OmniverseKit_Persp does not stick: with ClearXformOpOrder it raises
        # "already exists" (the attributes outlive the order, which is lecture
        # 3.2's point), and setting the existing ops reads back as Kit's default
        # (5, 5, 5) because Kit's own camera manipulator owns that prim and
        # writes its state back. A camera this script creates has no such owner.
        cam = UsdGeom.Camera.Define(stage, "/World/lecture_cam")
        cam.CreateFocalLengthAttr(24.0)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 100000.0))
        xf = UsdGeom.Xformable(cam.GetPrim())
        # TWO SHOTS, not one, and the reason is the lecture's own point: at a
        # range that fits the 64 m broken crate, the 0.6 m correct one is a
        # hundred times smaller and reads as nothing at all. So the take pulls
        # WIDE for the disaster and comes CLOSE for the comparison, and
        # CAM_SHOT picks which. One framing cannot serve both.
        #
        # The transform is BUILT WITH Gf.Matrix4d().SetLookAt rather than by
        # hand-solving Euler angles. Two hand-solved attempts put the camera
        # inside the crate, because rotateXYZ is applied to a camera that looks
        # down its own -Z and the azimuth/elevation intuition does not survive
        # that. A look-at matrix cannot get the convention wrong.
        import os
        wide = os.environ.get("CAM_SHOT", "wide") == "wide"
        if wide:
            eye, target = Gf.Vec3d(96.0, -46.0, 74.0), Gf.Vec3d(0.0, 78.0, 22.0)
        else:
            eye, target = Gf.Vec3d(2.3, -2.9, 1.5), Gf.Vec3d(0.0, 0.0, 0.28)
        m = Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse()
        xf.AddTransformOp().Set(m)
        vp = vpu.get_active_viewport()
        vp.camera_path = "/World/lecture_cam"
        print("FACT cam_set", vp.camera_path, "shot=%s" % ("wide" if wide else "tight"),
              "eye=", tuple(round(v, 1) for v in eye), flush=True)
    except Exception as e:
        print("FRAME_FAILED", e, flush=True)


asyncio.ensure_future(_frame())
print("USD_UNITS_READY", flush=True)
