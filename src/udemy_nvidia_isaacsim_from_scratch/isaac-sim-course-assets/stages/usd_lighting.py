# 3.6 FAILURE: a referenced asset brings its own DomeLight and takes over the
# scene's lighting.
#
# This has to be REPRODUCED, not drawn. The vendor asset is authored here with a
# DomeLight inside it, exactly the way a real asset ships one so it looks right
# in the vendor's own turntable. Reference that asset into a lit scene and the
# lighting changes, because a DomeLight is a prim like any other and composition
# brings it in with everything else.
#
# CAM_SHOT / LIGHT_STATE switch what the take records, from ONE stage script:
#   LIGHT_STATE=before   the scene as you lit it, asset not yet referenced
#   LIGHT_STATE=after    the same scene with the asset referenced. Two domes.
#   LIGHT_STATE=fixed    the asset's dome deactivated, yours back in control
import os

import omni.usd
from pxr import Usd, UsdGeom, UsdLux, Sdf, Gf

VENDOR = "/workspace/out/vendor_lamp_asset.usda"

# The scene's own lighting: a cool, dim dome, deliberately unlike the vendor's.
SCENE_DOME_INTENSITY = 500.0
SCENE_DOME_COLOR = (0.42, 0.58, 1.00)      # cool blue, a "studio" look

# The vendor's: hot, bright, orange. A sunset HDRI look, which is exactly the
# kind of thing an asset ships to flatter itself.
#
# The separation between these two is LOAD-BEARING, not taste. At 900 against
# 4200 with gentler colours the three states measured luma 189.2 / 201.2 / 187.1:
# a real change, and far too subtle for a viewer to see in a video whose entire
# subject is that change. Dome light contributions are additive, so what matters
# is the RATIO and the colour distance, and both are now wide.
VENDOR_DOME_INTENSITY = 9000.0
VENDOR_DOME_COLOR = (1.00, 0.45, 0.08)


def build_vendor_asset():
    """A crate that ships WITH ITS OWN DOME LIGHT, as many real assets do."""
    layer = Sdf.Layer.CreateNew(VENDOR)
    sub = Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(sub, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(sub, 1.0)
    root = UsdGeom.Xform.Define(sub, "/lamp_asset")
    sub.SetDefaultPrim(root.GetPrim())
    body = UsdGeom.Cube.Define(sub, "/lamp_asset/body")
    body.CreateSizeAttr(1.0)
    UsdGeom.Xformable(body).AddScaleOp().Set(Gf.Vec3f(0.35, 0.35, 0.5))
    body.CreateDisplayColorAttr([(0.55, 0.38, 0.18)])
    # THE OFFENDER. Inside the asset, not in your scene.
    dome = UsdLux.DomeLight.Define(sub, "/lamp_asset/asset_dome")
    dome.CreateIntensityAttr(VENDOR_DOME_INTENSITY)
    dome.CreateColorAttr(Gf.Vec3f(*VENDOR_DOME_COLOR))
    layer.Save()
    return layer


build_vendor_asset()
if not Sdf.Layer.FindOrOpen(VENDOR):
    raise RuntimeError(f"{VENDOR} was not written; the reference would resolve "
                       "to nothing and there would be no second light to show")

STATE = os.environ.get("LIGHT_STATE", "after")

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(ground)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
gx.AddScaleOp().Set(Gf.Vec3f(24, 24, 0.1))
ground.CreateDisplayColorAttr([(0.72, 0.74, 0.76)])

# a few native props, so the light change is visible on things you DO own
for i, x in enumerate((-1.4, -0.5, 0.6)):
    c = UsdGeom.Cube.Define(stage, f"/World/prop_{i}")
    c.CreateSizeAttr(1.0)
    cx = UsdGeom.Xformable(c)
    cx.AddTranslateOp().Set(Gf.Vec3d(x, 0.4, 0.22))
    cx.AddScaleOp().Set(Gf.Vec3f(0.44, 0.44, 0.44))
    c.CreateDisplayColorAttr([(0.78, 0.79, 0.80)])

# KILL ISAAC'S OWN DEFAULT LIGHT FIRST.
#
# Kit creates /Environment/defaultLight, a DistantLight at intensity 3000, on
# every new stage. It washes the scene to near-white and both domes below then
# have almost no visible effect: measured, the "before", "after" and "fixed"
# states came out at mean luma 226.6 / 227.1 / 225.0, which is no difference at
# all for a lecture whose entire subject is a lighting change. It is also the
# honest setup for a lighting lecture: the lights you can see should be the
# lights that are lighting it.
# Deactivate every DistantLight Kit created outside /World, whatever it is
# called: the path has changed between Kit versions and a hardcoded one that
# misses fails SILENTLY, leaving the scene washed out and the lecture pointless.
killed = []
for prim in stage.Traverse():
    path = prim.GetPath().pathString
    if prim.IsA(UsdLux.DistantLight) and not path.startswith("/World"):
        prim.SetActive(False)
        killed.append(path)
print("FACT killed_default_lights", killed, flush=True)

# YOUR lighting. Cool and dim: the look you set up on purpose.
my_dome = UsdLux.DomeLight.Define(stage, "/World/my_dome")
my_dome.CreateIntensityAttr(SCENE_DOME_INTENSITY)
my_dome.CreateColorAttr(Gf.Vec3f(*SCENE_DOME_COLOR))

if STATE in ("after", "fixed"):
    lamp = UsdGeom.Xform.Define(stage, "/World/vendor_lamp")
    lamp.GetPrim().GetReferences().AddReference(VENDOR)
    x = UsdGeom.Xformable(lamp)
    for op in x.GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            op.Set(Gf.Vec3d(1.5, 0.4, 0.25))
            break
    else:
        x.AddTranslateOp().Set(Gf.Vec3d(1.5, 0.4, 0.25))

if STATE == "fixed":
    # THE FIX, and it is deliberately not "delete their light": you do not own
    # that file, and 3.3 established that editing it changes every scene that
    # references it. Deactivating the prim in YOUR layer is an opinion on top,
    # which is exactly what a layer is for.
    p = stage.GetPrimAtPath("/World/vendor_lamp/asset_dome")
    if p:
        p.SetActive(False)
        print("FACT deactivated /World/vendor_lamp/asset_dome", flush=True)

# Report every light the stage actually has, so the take can be checked against
# the runtime rather than against this file's intent.
lights = []
for prim in stage.Traverse():
    if prim.IsA(UsdLux.DomeLight):
        d = UsdLux.DomeLight(prim)
        lights.append((str(prim.GetPath()), prim.IsActive(),
                       d.GetIntensityAttr().Get(), tuple(d.GetColorAttr().Get())))
print("FACT state", STATE, flush=True)
for p, active, i, c in lights:
    print(f"FACT dome {p} active={active} intensity={i} color={c}", flush=True)
print("FACT dome_count", len(lights), flush=True)

import asyncio
import omni.kit.app


async def _frame():
    """Own camera + look-at matrix + repointed viewport. See usd_units.py: the
    default camera prim cannot be written reliably here."""
    app = omni.kit.app.get_app()
    for _ in range(700):
        await app.next_update_async()
    try:
        import omni.kit.viewport.utility as vpu
        cam = UsdGeom.Camera.Define(stage, "/World/lecture_cam")
        cam.CreateFocalLengthAttr(28.0)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 10000.0))
        xf = UsdGeom.Xformable(cam.GetPrim())
        m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(1.2, -4.6, 2.2),
                                    Gf.Vec3d(0.0, 0.3, 0.25),
                                    Gf.Vec3d(0, 0, 1)).GetInverse()
        xf.AddTransformOp().Set(m)
        vpu.get_active_viewport().camera_path = "/World/lecture_cam"
        print("FACT cam_set /World/lecture_cam", flush=True)
    except Exception as e:
        print("FRAME_FAILED", e, flush=True)


asyncio.ensure_future(_frame())
print("USD_LIGHTING_READY", flush=True)
