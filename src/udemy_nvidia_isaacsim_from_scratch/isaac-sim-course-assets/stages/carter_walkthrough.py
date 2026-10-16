# Carter at REAL scale in a NORMAL Isaac Sim window, framed properly.
#
# Ferbin: "have normal windows and make sure to walk through the steps for doing
# it properly as well". So no scale trick: the robot is 0.9 m, the window keeps
# its panels and Stage tree, and the camera is placed deliberately.
#
# The framing route that works, after five that did not: set the viewport camera
# AFTER the app has fully started, from a callback on the stage-open event rather
# than from the --exec script body. The earlier attempts all ran too early and
# Kit's own startup framing overwrote them.
import asyncio
import omni.kit.app
import omni.usd
from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, PhysxSchema, Gf

CARTER = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
          "/Assets/Isaac/4.5/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd")

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr().Set(9.81)
psx = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
psx.CreateSolverTypeAttr("TGS")
psx.CreateTimeStepsPerSecondAttr(120)

ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(ground)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
gx.AddScaleOp().Set(Gf.Vec3f(30.0, 30.0, 0.1))
ground.CreateDisplayColorAttr([(0.60, 0.62, 0.64)])
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

rover = UsdGeom.Xform.Define(stage, "/World/carter")
rover.GetPrim().GetReferences().AddReference(CARTER)
# REAL SCALE. No rig scaling: a learner must see 0.9 m of robot as 0.9 m.

dome = UsdLux.DomeLight.Define(stage, "/World/dome")
dome.CreateIntensityAttr(1100.0)


async def _frame():
    """Place the viewport camera once the app is genuinely up.

    Five earlier routes lost to Kit's startup ordering. Waiting for the app to
    settle, THEN writing the camera transform, is the one that sticks, because
    there is no longer a startup pass left to overwrite it.
    """
    app = omni.kit.app.get_app()
    for _ in range(700):                       # ~ the cloud reference resolving
        await app.next_update_async()
    try:
        import omni.kit.viewport.utility as vpu
        vp = vpu.get_active_viewport()
        cam_path = vp.camera_path
        cam = stage.GetPrimAtPath(str(cam_path))
        xf = UsdGeom.Xformable(cam)
        xf.ClearXformOpOrder()
        # a normal working view: back far enough to see the robot AND its floor
        xf.AddTranslateOp().Set(Gf.Vec3d(2.35, -2.95, 1.55))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(68.0, 0.0, 38.0))
        print("[carter] viewport camera placed on", cam_path)
    except Exception as exc:
        print("[carter] framing failed:", exc)


asyncio.ensure_future(_frame())
print("[carter] real-scale scene built, framing scheduled")
