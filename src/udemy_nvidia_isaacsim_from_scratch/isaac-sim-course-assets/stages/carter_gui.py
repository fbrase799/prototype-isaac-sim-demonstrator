# Carter in the INTERACTIVE viewport, framed with Kit's own "frame selected"
# command rather than by setting camera transforms.
#
# Why this way. The viewport renders Carter sharply already; the only unsolved
# part was framing. Three attempts to set a camera transform were overridden by
# Kit's startup, and the offscreen Replicator route turned out to be writing its
# own idle frames rather than mine (rep.orchestrator.step raises inside --exec).
#
# So: stop fighting the camera and ask the APPLICATION to frame the robot, which
# is the same thing a user does by pressing F. Selection plus
# "omni.kit.viewport.actions.frame_selected_prims" is Kit's own supported path
# and it runs after the stage exists, so there is no startup race to lose.
import asyncio
import omni.kit.app
import omni.kit.commands
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

# a light studio floor, matching the course look
ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(ground)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
gx.AddScaleOp().Set(Gf.Vec3f(60.0, 60.0, 0.1))
ground.CreateDisplayColorAttr([(0.62, 0.64, 0.66)])
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
gm = UsdPhysics.MaterialAPI.Apply(ground.GetPrim())
gm.CreateStaticFrictionAttr(1.0)
gm.CreateDynamicFrictionAttr(0.9)
gm.CreateRestitutionAttr(0.0)

rover = UsdGeom.Xform.Define(stage, "/World/carter")
rover.GetPrim().GetReferences().AddReference(CARTER)


def xform_op(xf, kind):
    """Referenced prims arrive with their own xformOpOrder, so AddTranslateOp
    throws 'already exists'. Reuse what is there."""
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == kind:
            return op
    return {UsdGeom.XformOp.TypeTranslate: xf.AddTranslateOp,
            UsdGeom.XformOp.TypeRotateZ: xf.AddRotateZOp}[kind]()


rx = UsdGeom.Xformable(rover)
T = xform_op(rx, UsdGeom.XformOp.TypeTranslate)
T.Set(Gf.Vec3d(0, 0, 0))
# fill Kit's default framing by scaling the rig rather than moving the camera
SCALE = 3.4
for _op in rx.GetOrderedXformOps():
    if _op.GetOpType() == UsdGeom.XformOp.TypeScale:
        _op.Set(Gf.Vec3f(SCALE, SCALE, SCALE))
        break
else:
    rx.AddScaleOp().Set(Gf.Vec3f(SCALE, SCALE, SCALE))
# the floor scales with it so the proportions stay believable
gx.GetOrderedXformOps()[1].Set(Gf.Vec3f(60.0*SCALE, 60.0*SCALE, 0.1*SCALE))

dome = UsdLux.DomeLight.Define(stage, "/World/dome")
dome.CreateIntensityAttr(1100.0)


async def _setup():
    app = omni.kit.app.get_app()
    # let the cloud reference resolve and shaders warm
    for _ in range(600):
        await app.next_update_async()
    # select the robot and ask Kit to frame it, exactly as pressing F does
    ctx = omni.usd.get_context()
    ctx.get_selection().set_selected_prim_paths(["/World/carter"], True)
    for _ in range(20):
        await app.next_update_async()
    try:
        omni.kit.commands.execute("FramePrimsCommand",
                                  prim_to_move=None,
                                  prims_to_frame=["/World/carter"],
                                  time_code=Usd.TimeCode.Default(),
                                  usd_context_name="",
                                  aspect_ratio=1.777)
        print("[carter] FramePrimsCommand issued")
    except Exception as exc:
        print("[carter] FramePrimsCommand failed:", exc)
        try:
            import omni.kit.viewport.utility as vpu
            vpu.frame_viewport_selection(vpu.get_active_viewport())
            print("[carter] frame_viewport_selection issued")
        except Exception as exc2:
            print("[carter] both framing routes failed:", exc2)
    for _ in range(30):
        await app.next_update_async()
    # deselect, so no highlight outline appears in the footage
    ctx.get_selection().set_selected_prim_paths([], True)
    print("[carter] ready to record")


asyncio.ensure_future(_setup())
print("[carter] scene built, framing scheduled")
