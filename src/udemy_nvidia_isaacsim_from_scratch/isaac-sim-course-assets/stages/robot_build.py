# Section 5: the robot, built up in stages.
#
# One script, one robot, parameterised by BUILD_STAGE so each lecture can
# capture the SAME machine at its own point in the build. That matters because
# the section's promise is "from File > New to a driveable chassis": if each
# lecture built a different robot the continuity would be a lie.
#
#   ground     5.1  an empty stage and a ground plane
#   chassis    5.2  the body, at real dimensions
#   wheels     5.3  two driven wheels on revolute joints
#   caster     5.4  the third contact point
#   art        5.5  the articulation root
#   drives     5.6  joint drives configured
#   vibrate    5.7  the FAILURE: stiffness the timestep cannot keep up with
#   drive      5.8  driving it, reading the joint states back
#
# Real dimensions, chosen to match a small indoor AMR and stated on camera:
#   chassis 0.60 x 0.40 x 0.18 m, 12 kg
#   wheels  0.10 m radius, 0.04 m wide, on the centre line
#   caster  0.045 m sphere at the front
import os
import omni.usd
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf, UsdLux, UsdShade

STAGE_NAME = os.environ.get("BUILD_STAGE", "drive")
ORDER = ["ground", "chassis", "wheels", "caster", "art", "drives", "vibrate", "drive", "camera"]
LEVEL = ORDER.index(STAGE_NAME) if STAGE_NAME in ORDER else len(ORDER) - 1


def at_least(name):
    return LEVEL >= ORDER.index(name)


CH_L, CH_W, CH_H = 0.60, 0.40, 0.18
CH_MASS = 12.0
WH_R, WH_W = 0.10, 0.04
CASTER_R = 0.055
# Forward of the chassis nose (0.30), so the caster is VISIBLE. Tucked at 0.22
# it sat entirely inside the 0.10 m ground clearance and no camera angle could
# see it, which is useless for the lecture that is about it.
CASTER_X = 0.335
DECK_Z = WH_R                       # axle height sets the deck height

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)

# a physics material with grip, or the wheels spin uselessly (4.6's lesson)
mat = UsdShade.Material.Define(stage, "/World/physMat")
pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
pm.CreateStaticFrictionAttr(0.9)
pm.CreateDynamicFrictionAttr(0.85)
pm.CreateRestitutionAttr(0.0)


def bind(prim):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        mat, UsdShade.Tokens.weakerThanDescendants, "physics")


# --- 5.1 ground -------------------------------------------------------------
ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(ground)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
gx.AddScaleOp().Set(Gf.Vec3f(6, 6, 0.1) if STAGE_NAME == "ground"
                    else Gf.Vec3f(20, 20, 0.1))
ground.CreateDisplayColorAttr([(0.62, 0.64, 0.66)])
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
bind(ground.GetPrim())

UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(1100.0)

if STAGE_NAME == "ground":
    # 5.1 ends by VERIFYING the stage with one drop: a cube that lands proves
    # the collider, the gravity and the units all at once. Without it the shot
    # is an empty grid, which is correct and teaches nothing.
    probe = UsdGeom.Cube.Define(stage, "/World/drop_test")
    probe.CreateSizeAttr(0.30)
    UsdGeom.Xformable(probe).AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.6))
    probe.CreateDisplayColorAttr([(0.35, 0.62, 0.05)])
    UsdPhysics.CollisionAPI.Apply(probe.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(probe.GetPrim())
    UsdPhysics.MassAPI.Apply(probe.GetPrim()).CreateMassAttr(2.0)
    bind(probe.GetPrim())

robot = None
if at_least("chassis"):
    robot = UsdGeom.Xform.Define(stage, "/World/robot")
    UsdGeom.Xformable(robot).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))

    body = UsdGeom.Cube.Define(stage, "/World/robot/chassis")
    body.CreateSizeAttr(1.0)
    bx = UsdGeom.Xformable(body)
    bx.AddTranslateOp().Set(Gf.Vec3d(0, 0, DECK_Z + CH_H / 2))
    bx.AddScaleOp().Set(Gf.Vec3f(CH_L, CH_W, CH_H))
    body.CreateDisplayColorAttr([(0.35, 0.40, 0.46)])
    UsdPhysics.CollisionAPI.Apply(body.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(CH_MASS)
    bind(body.GetPrim())

WHEELS = [("wheel_left", 1), ("wheel_right", -1)]
if at_least("wheels"):
    for name, side in WHEELS:
        w = UsdGeom.Cylinder.Define(stage, f"/World/robot/{name}")
        w.CreateRadiusAttr(WH_R)
        w.CreateHeightAttr(WH_W)
        w.CreateAxisAttr("Y")

        wx = UsdGeom.Xformable(w)
        wx.AddTranslateOp().Set(Gf.Vec3d(-0.12, side * (CH_W / 2 + WH_W / 2), WH_R))
        w.CreateDisplayColorAttr([(0.16, 0.17, 0.19)])
        UsdPhysics.CollisionAPI.Apply(w.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(w.GetPrim())
        UsdPhysics.MassAPI.Apply(w.GetPrim()).CreateMassAttr(0.8)
        bind(w.GetPrim())

        j = UsdPhysics.RevoluteJoint.Define(stage, f"/World/robot/joint_{name}")
        j.CreateAxisAttr("Y")
        j.CreateBody0Rel().SetTargets([body.GetPrim().GetPath()])
        j.CreateBody1Rel().SetTargets([w.GetPrim().GetPath()])
        # Local frames in each body's OWN space, scaled by its own size: the
        # trap from 4.8. The chassis is a unit cube scaled to CH_L x CH_W x CH_H,
        # so an offset of 1.0 in x is CH_L metres.
        j.CreateLocalPos0Attr(Gf.Vec3f(-0.12 / CH_L,
                                       side * (CH_W / 2 + WH_W / 2) / CH_W,
                                       (WH_R - DECK_Z - CH_H / 2) / CH_H))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))

if at_least("caster"):
    c = UsdGeom.Sphere.Define(stage, "/World/robot/caster")
    c.CreateRadiusAttr(CASTER_R)
    UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(CASTER_X, 0, CASTER_R))
    c.CreateDisplayColorAttr([(0.55, 0.57, 0.60)])
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(c.GetPrim())
    UsdPhysics.MassAPI.Apply(c.GetPrim()).CreateMassAttr(0.3)
    bind(c.GetPrim())
    # A ball caster is a SPHERE on a fixed joint: it rolls on its own surface,
    # so it needs no joint of its own. 5.4 is about why the obvious
    # alternatives (a second revolute, or a swivel) are harder than this.
    jc = UsdPhysics.FixedJoint.Define(stage, "/World/robot/joint_caster")
    jc.CreateBody0Rel().SetTargets([body.GetPrim().GetPath()])
    jc.CreateBody1Rel().SetTargets([c.GetPrim().GetPath()])
    jc.CreateLocalPos0Attr(Gf.Vec3f(CASTER_X / CH_L, 0,
                                    (CASTER_R - DECK_Z - CH_H / 2) / CH_H))
    jc.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))

if at_least("art"):
    # THE most-missed step. Without an articulation root the joints are treated
    # as independent constraints rather than one articulated system, and the
    # machine is both slower and less stable.
    PhysxSchema.PhysxArticulationAPI.Apply(robot.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

STIFF = float(os.environ.get("DRIVE_STIFF", "0"))
DAMP = float(os.environ.get("DRIVE_DAMP", "1200"))
TARGET_VEL = float(os.environ.get("DRIVE_VEL", "0"))
if at_least("drives"):
    if STAGE_NAME == "vibrate":
        # 5.7's failure: a stiffness far beyond what the timestep can resolve.
        STIFF, DAMP = 4.0e7, 0.0
    for name, side in WHEELS:
        j = stage.GetPrimAtPath(f"/World/robot/joint_{name}")
        d = UsdPhysics.DriveAPI.Apply(j, "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(STIFF)
        d.CreateDampingAttr(DAMP)
        d.CreateMaxForceAttr(1.0e4)
        if at_least("drive") or STAGE_NAME == "vibrate":
            d.CreateTargetVelocityAttr(TARGET_VEL if TARGET_VEL else 240.0)

# --- the shot ---------------------------------------------------------------
# Framing solved as in Section 4: the robot spans about 0.7 m, so a half-width
# of 0.75 m needs 1.72 m at 24 mm. 2.4 m is used so the ground plane reads too.
cam = UsdGeom.Camera.Define(stage, "/World/shot_cam")
cam.CreateFocalLengthAttr(24.0)
# The nose is +x (the caster is at x = +0.335), so the camera must be on the
# +x side to see it. Two framings were wasted before checking which way the
# robot faced: (0.95, -1.85, 0.85) looked from behind-right and hid the far
# wheel and the caster entirely, and (-1.30, -1.15, 0.52) looked at the TAIL,
# which put the caster behind the chassis as a sliver. From +x and low, the
# caster leads, both wheels straddle the body and the deck still reads.
eye = Gf.Vec3d(1.35, -1.05, 0.46)
if STAGE_NAME == "camera":
    # Section 6 needs the robot AND what it is looking at in the same shot.
    # At the build framing the obstacles sit 2.2 to 3.1 m ahead, entirely
    # outside the frame, so the sensor lecture filmed a robot staring at
    # nothing. Pulled back and swung round to see down its line of sight.
    eye = Gf.Vec3d(-1.6, -3.6, 1.9)
if STAGE_NAME in ("drive", "vibrate"):
    # The DRIVING stages need a wider shot: at the build framing the robot
    # leaves frame within a second and every sampled frame caught it half off
    # the right edge. Pulled back and raised so it stays in shot for the whole
    # run, at the cost of the close detail those lectures do not need.
    eye = Gf.Vec3d(2.4, -3.4, 1.5)
if STAGE_NAME == "ground":
    eye = Gf.Vec3d(3.1, -4.4, 2.30)
target = Gf.Vec3d(1.6, 0, 0.20) if STAGE_NAME == "camera" else Gf.Vec3d(0.02, 0, 0.13)
m = Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse()
UsdGeom.Xformable(cam).AddTransformOp().Set(m)

try:
    import omni.kit.viewport.utility as vp_util
    vp = vp_util.get_active_viewport()
    if vp is not None:
        vp.camera_path = "/World/shot_cam"
except Exception as exc:                       # noqa: BLE001
    print(f"  viewport camera not set: {exc}", flush=True)

if at_least("camera"):
    # Section 6 mounts the sensor as a CHILD of the chassis so it inherits the
    # chassis transform. Mounted as a sibling it stays where you left it while
    # the robot drives away, which is 6.1's warning.
    eye = UsdGeom.Camera.Define(stage, "/World/robot/chassis/eye")
    eye.CreateFocalLengthAttr(24.0)
    eye.CreateHorizontalApertureAttr(20.955)
    eye.CreateVerticalApertureAttr(20.955 * 9 / 16)
    # look forward (+x) from the front face of the chassis, in CHASSIS space
    em = Gf.Matrix4d().SetLookAt(Gf.Vec3d(0.45, 0, 0.2),
                                 Gf.Vec3d(3.0, 0, 0.0),
                                 Gf.Vec3d(0, 0, 1)).GetInverse()
    UsdGeom.Xformable(eye).AddTransformOp().Set(em)
    # something to look AT, or the sensor lecture films an empty plane
    for i, (ox, oy, h) in enumerate(((2.2, 0.0, 0.30), (3.1, 0.7, 0.45), (2.6, -0.8, 0.22))):
        ob = UsdGeom.Cube.Define(stage, f"/World/obstacle_{i}")
        ob.CreateSizeAttr(1.0)
        obx = UsdGeom.Xformable(ob)
        obx.AddTranslateOp().Set(Gf.Vec3d(ox, oy, h / 2))
        obx.AddScaleOp().Set(Gf.Vec3f(h, h, h))
        ob.CreateDisplayColorAttr([(0.35, 0.62, 0.05) if i == 0 else (0.75, 0.76, 0.78)])
        UsdPhysics.CollisionAPI.Apply(ob.GetPrim())
        bind(ob.GetPrim())

print(f"ROBOT_BUILD_READY stage={STAGE_NAME} level={LEVEL}", flush=True)
for p in stage.Traverse():
    if p.GetPath().pathString.startswith("/World/robot"):
        print(f"  {p.GetPath()}  {list(p.GetAppliedSchemas())}", flush=True)

if at_least("drive") or STAGE_NAME in ("vibrate", "ground"):
    try:
        import omni.timeline
        omni.timeline.get_timeline_interface().play()
        print("ROBOT_BUILD_PLAYING", flush=True)
        if STAGE_NAME == "ground":
            # re-drop every 4 s on the wall clock, so the take has repeated
            # landings rather than one event and 56 s of a resting cube
            import time as _t
            import omni.kit.app
            from pxr import UsdPhysics as _UP
            _pr = stage.GetPrimAtPath("/World/drop_test")
            _st = {"last": _t.monotonic()}

            def _redrop(e):
                now = _t.monotonic()
                if now - _st["last"] < 4.0:
                    return
                _st["last"] = now
                for op in UsdGeom.Xformable(_pr).GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        op.Set(Gf.Vec3d(0, 0, 1.6))
                rb = _UP.RigidBodyAPI(_pr)
                rb.GetVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
                rb.GetAngularVelocityAttr().Set(Gf.Vec3f(0, 0, 0))

            _sub = (omni.kit.app.get_app().get_update_event_stream()
                    .create_subscription_to_pop(_redrop, name="ground_redrop"))
    except Exception as exc:                   # noqa: BLE001
        print(f"  could not start the timeline: {exc}", flush=True)
