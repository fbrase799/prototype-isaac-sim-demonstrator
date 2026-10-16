# 2.9: prove rendering happens with NO window and NO DISPLAY, without Replicator.
#
# Replicator's writer path is a dead end on this image: omni.replicator.replicator_yaml
# fails at startup, which leaves the writers unregistered, then the storage backends
# unregistered, then the "DispatchSync" SDG node template unregistered. Three errors
# in sequence, each looking like your own code is wrong.
#
# The viewport capture API needs none of that, and it is what a learner would
# reach for anyway.
import asyncio, omni.kit.app, omni.usd
from pxr import UsdGeom, UsdLux, Gf

CARTER = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
          "/Assets/Isaac/4.5/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd")
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
w = UsdGeom.Xform.Define(stage, "/World"); stage.SetDefaultPrim(w.GetPrim())
g = UsdGeom.Cube.Define(stage, "/World/ground"); g.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(g); gx.AddTranslateOp().Set(Gf.Vec3d(0,0,-0.05))
gx.AddScaleOp().Set(Gf.Vec3f(30,30,0.1))
g.CreateDisplayColorAttr([(0.6,0.62,0.64)])
r = UsdGeom.Xform.Define(stage, "/World/carter")
r.GetPrim().GetReferences().AddReference(CARTER)
UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(1100.0)

async def go():
    app = omni.kit.app.get_app()
    for _ in range(600):
        await app.next_update_async()
    import omni.kit.viewport.utility as vpu
    vp = vpu.get_active_viewport()
    # place the camera deliberately, the same way the GUI takes do
    from pxr import Sdf
    cam = stage.GetPrimAtPath("/OmniverseKit_Persp")
    if cam:
        cam.GetAttribute("xformOp:translate").Set(Gf.Vec3d(3.4, -3.4, 2.1))
        rot = cam.GetAttribute("xformOp:rotateYXZ") or cam.GetAttribute("xformOp:rotateXYZ")
        if rot: rot.Set(Gf.Vec3f(63, 0, 45))
    for _ in range(120):
        await app.next_update_async()
    out = "/workspace/out/headless/headless_frame.png"
    cap = vpu.capture_viewport_to_file(vp, file_path=out)
    for _ in range(240):
        await app.next_update_async()
    print("HEADLESS_CAPTURE_DONE", out, flush=True)

asyncio.ensure_future(go())
