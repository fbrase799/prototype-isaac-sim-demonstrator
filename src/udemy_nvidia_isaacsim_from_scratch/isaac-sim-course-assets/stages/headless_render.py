# 2.9: prove a render happens with NO window and NO DISPLAY.
# Writes a PNG from a camera using Replicator's synchronous writer path.
import asyncio, omni.kit.app, omni.usd
from pxr import UsdGeom, UsdLux, UsdPhysics, Gf

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

cam = UsdGeom.Camera.Define(stage, "/World/cam")
cx = UsdGeom.Xformable(cam)
cx.AddTranslateOp().Set(Gf.Vec3d(3.2, -3.2, 2.0))
cx.AddRotateXYZOp().Set(Gf.Vec3f(63, 0, 45))

async def go():
    app = omni.kit.app.get_app()
    for _ in range(500):
        await app.next_update_async()
    import omni.replicator.core as rep
    # 6.0 does not have BasicWriter in the registry at this point: the
    # replicator_yaml extension fails at startup and the default writers module
    # is never imported. Import it explicitly and register what we need.
    # The omni.replicator.replicator_yaml extension fails to start in 6.0
    # ("'NoneType' object has no attribute '_unregister_status_callback'"), and
    # it is what normally imports the default writers AND registers the storage
    # backends. Without it you get two errors in sequence that both look like
    # your code is wrong: "No writer named BasicWriter" and then "No backend
    # could be initialized". Do both registrations explicitly.
    try:
        from omni.replicator.core.scripts.backends import register_backends
        register_backends()
        from omni.replicator.core.scripts.writers_default import BasicWriter
        rep.WriterRegistry.register(BasicWriter)
        print("REGISTERED writers+backends", flush=True)
    except Exception as e:
        print("REGISTER_FALLBACK", e, flush=True)
    rp = rep.create.render_product("/World/cam", (1280, 720))
    wr = rep.WriterRegistry.get("BasicWriter")
    wr.initialize(output_dir="/workspace/out/headless", rgb=True)
    wr.attach([rp])
    for _ in range(60):
        await rep.orchestrator.step_async()
    print("HEADLESS_RENDER_DONE", flush=True)

asyncio.ensure_future(go())
