# The hero robot for real-Isaac footage: NVIDIA's Nova Carter, which is a REAL
# production AMR with proper geometry and materials, referenced from the official
# Isaac asset library.
#
# Why not the hand-built one. Ferbin: "the robot in sim is tooo simple its just
# circle and rectangle I need good looking robots". He is right: a scaled Cube
# with four Cylinders is a primitive, and no amount of extra child boxes makes it
# a machine. The course still TEACHES building a robot from scratch (that is
# section 5, and the hand-built USD stays for it), but footage whose job is to
# look like a real robot should use a real robot.
from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, Gf, Sdf
import omni.usd

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

# ground
ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(ground)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
gx.AddScaleOp().Set(Gf.Vec3f(60.0, 60.0, 0.1))
ground.CreateDisplayColorAttr([(0.42, 0.44, 0.47)])
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

# the robot, as a REFERENCE. This is also lecture 3.3's subject: a stage that
# points at another description rather than containing it.
rover = UsdGeom.Xform.Define(stage, "/World/carter")
rover.GetPrim().GetReferences().AddReference(CARTER)
UsdGeom.Xformable(rover).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.0))

# lighting: a key and a soft dome, so the materials actually show
dome = UsdLux.DomeLight.Define(stage, "/World/dome")
dome.CreateIntensityAttr(1000.0)
key = UsdLux.DistantLight.Define(stage, "/World/key")
key.CreateIntensityAttr(3000.0)
key.CreateAngleAttr(1.2)
UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-40, 0, 42))

# Frame the DEFAULT perspective camera, which is the one the viewport already
# uses. Switching vp.camera_path to a custom camera reverted to Perspective both
# immediately and on a deferred tick, so this stops fighting the startup order.
for _p in ("/OmniverseKit_Persp",):
    _c = stage.GetPrimAtPath(_p)
    if _c:
        _x = UsdGeom.Xformable(_c)
        _x.ClearXformOpOrder()
        _x.AddTranslateOp().Set(Gf.Vec3d(1.30, -1.70, 0.68))
        _x.AddRotateXYZOp().Set(Gf.Vec3f(79, 0, 38))
        print("[carter] framed the perspective camera")

# a shot camera is still defined, for offscreen renders that ask for it by name
cam = UsdGeom.Camera.Define(stage, "/World/shotCam")
cx = UsdGeom.Xformable(cam)
cx.AddTranslateOp().Set(Gf.Vec3d(1.35, -1.75, 0.72))
cx.AddRotateXYZOp().Set(Gf.Vec3f(80, 0, 38))
cam.CreateFocalLengthAttr(38.0)
# The viewport reverts to Perspective if it is told about a camera before the
# referenced payload has resolved, so set it on a later update tick.
def _set_cam(*_a):
    try:
        import omni.kit.viewport.utility as vpu
        vp = vpu.get_active_viewport()
        if vp is not None:
            vp.camera_path = "/World/shotCam"
            print("[carter] shotCam active")
    except Exception as exc:
        print("[carter] camera framing skipped:", exc)
try:
    import omni.kit.app, carb
    _sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        lambda e, n=[0]: (n.__setitem__(0, n[0]+1), _set_cam() if n[0] == 240 else None))
except Exception:
    _set_cam()

print("[carter] referenced:", CARTER.rsplit('/', 1)[-1])
print("[carter] prims:", len(list(stage.Traverse())))
