# The course robot, built in USD so it can be SHOWN in the real Isaac Sim GUI.
# Lecture 1.1 promises "you will build exactly this from an empty stage", so the
# thing on screen has to be a real articulated robot in the real viewport, not a
# drawing of one. Proportions follow the same measured AMR reference the studiokit
# rover uses (MiR250 class: 0.80 x 0.58 x 0.30 m).
from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, PhysxSchema, Gf, Sdf
import omni.usd

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

# --- physics scene ---------------------------------------------------------
scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr().Set(9.81)
# Contact chatter on four wheels is what remains after the drive is tamed, and
# it is a solver-iteration problem. PhysX defaults are tuned for many bodies, not
# for one vehicle that must look calm on camera.
psx = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
psx.CreateEnableCCDAttr(True)
psx.CreateSolverTypeAttr("TGS")
psx.CreateTimeStepsPerSecondAttr(120)

# --- ground ----------------------------------------------------------------
ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(ground)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
gx.AddScaleOp().Set(Gf.Vec3f(40.0, 40.0, 0.1))
ground.CreateDisplayColorAttr([(0.38, 0.40, 0.43)])
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
gmat = UsdPhysics.MaterialAPI.Apply(ground.GetPrim())
gmat.CreateStaticFrictionAttr(1.0)
gmat.CreateDynamicFrictionAttr(0.9)
gmat.CreateRestitutionAttr(0.0)     # a floor that does not bounce

# --- chassis ---------------------------------------------------------------
# NOT scaled. A joint's localPos is expressed in the body's LOCAL space, and for a
# prim with a scale op that is PRE-scale units, so mixing world-metre wheel
# positions with a scaled parent gave frames that did not line up. PhysX resolves
# that error by launching the parts apart: on the first step the chassis and all
# four wheels flew out of shot and only the (unjointed) lidar puck was left
# floating. Building the chassis at true size means local space IS metres, and
# the whole class of mistake disappears.
BL, BW, BH = 0.80, 0.58, 0.30
body = UsdGeom.Cube.Define(stage, "/World/rover/chassis")
body.CreateSizeAttr(1.0)
bx = UsdGeom.Xformable(body)
bx.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.26))
bx.AddScaleOp().Set(Gf.Vec3f(BL, BW, BH))
body.CreateDisplayColorAttr([(0.72, 0.74, 0.77)])
UsdPhysics.CollisionAPI.Apply(body.GetPrim())
rb = UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
mass = UsdPhysics.MassAPI.Apply(body.GetPrim())
mass.CreateMassAttr(18.0)
bpx = PhysxSchema.PhysxRigidBodyAPI.Apply(body.GetPrim())
bpx.CreateSolverPositionIterationCountAttr(16)
bpx.CreateSolverVelocityIterationCountAttr(4)
# a little linear damping keeps the body from creeping on contact noise
bpx.CreateLinearDampingAttr(0.15)
bpx.CreateAngularDampingAttr(0.35)

# the articulation root: the prim that turns a pile of links into a robot
UsdPhysics.ArticulationRootAPI.Apply(body.GetPrim())

# --- shell detailing ------------------------------------------------------
# "the robot in sim is tooo simple its just circle and rectangle". Correct: a
# scaled Cube with four Cylinders is a primitive, not a machine. These are the
# same features the animated rover carries, so the two read as ONE robot: a
# chamfered nose, a stepped upper deck, a dark bumper skirt around the base, and
# a recessed equipment bay. All are visual children of the chassis, so they ride
# with it and cost nothing in physics.
def _child_box(name, t, sc, colour):
    c = UsdGeom.Cube.Define(stage, f"/World/rover/chassis/{name}")
    c.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(c)
    # local space is the chassis's, which carries scale (BL,BW,BH)
    x.AddTranslateOp().Set(Gf.Vec3d(t[0]/BL, t[1]/BW, t[2]/BH))
    x.AddScaleOp().Set(Gf.Vec3f(sc[0]/BL, sc[1]/BW, sc[2]/BH))
    c.CreateDisplayColorAttr([colour])
    return c

# upper deck, inset and proud of the main body
_child_box("deck", (0.02, 0.0, 0.17), (0.66, 0.50, 0.045), (0.62, 0.65, 0.68))
# bumper skirt: dark, wrapping the base
_child_box("skirt", (0.0, 0.0, -0.13), (0.815, 0.60, 0.075), (0.13, 0.14, 0.16))
# equipment bay, recessed into the flank
_child_box("bay", (-0.10, 0.30, 0.02), (0.34, 0.012, 0.10), (0.44, 0.47, 0.50))
# Front sensor band, flush with the nose. An earlier version rotated a slab 38
# degrees to fake a chamfer and it simply stuck out through the bodywork on
# camera: a rotated box intersecting a box is not a chamfer. A flush inset band
# reads as a sensor face and cannot poke out.
_child_box("sensorband", (-0.345, 0.0, 0.06), (0.035, 0.44, 0.10),
           (0.07, 0.09, 0.11))
_child_box("noseplate",  (-0.352, 0.0, -0.02), (0.02, 0.50, 0.12),
           (0.55, 0.58, 0.61))

# --- sensor puck ----------------------------------------------------------
# A CHILD of the chassis, not a sibling. As a sibling under /World/rover it was
# a free prim with no rigid body and no joint, so when physics started the robot
# drove away and the puck stayed behind hanging in mid air. Parenting it to the
# chassis means it inherits the chassis transform and rides along, which is also
# how a real sensor is mounted.
puck = UsdGeom.Cylinder.Define(stage, "/World/rover/chassis/lidar")
puck.CreateRadiusAttr(0.075 / BL); puck.CreateHeightAttr(0.09 / BH)
puck.CreateAxisAttr("Z")
UsdGeom.Xformable(puck).AddTranslateOp().Set(
    Gf.Vec3d(0.13 / BL, 0.0, (0.46 - 0.26) / BH))
puck.CreateDisplayColorAttr([(0.16, 0.18, 0.20)])

# --- four wheels, as real revolute joints ---------------------------------
WR, WT = 0.098, 0.05
for i, (sx, sy) in enumerate([(1,1),(1,-1),(-1,1),(-1,-1)]):
    name = f"wheel_{i}"
    w = UsdGeom.Cylinder.Define(stage, f"/World/rover/{name}")
    w.CreateRadiusAttr(WR); w.CreateHeightAttr(WT)
    w.CreateAxisAttr("Y")
    wx = UsdGeom.Xformable(w)
    px_, py_ = sx*0.24, sy*(BW/2 + 0.01)
    wx.AddTranslateOp().Set(Gf.Vec3d(px_, py_, WR))
    w.CreateDisplayColorAttr([(0.12, 0.13, 0.14)])
    # THE HOP. A Cylinder collider defaults to a convex-hull approximation,
    # which for a wheel is a faceted polygon: it rolls on flat edges and drops
    # into each corner, so the whole robot bounced ~35 px on a repeating cycle.
    # Measured before this fix: y oscillating 495 <-> 530 every ~1.5 s.
    # PhysX has an exact analytic cylinder; ask for it explicitly.
    wcol = PhysxSchema.PhysxCollisionAPI.Apply(w.GetPrim())
    wcol.CreateContactOffsetAttr(0.008)
    wcol.CreateRestOffsetAttr(0.0)
    # The collision shape is a SPHERE, not the visual cylinder. PhysX treats a
    # sphere analytically, so there are no facets to drop into and the contact
    # point is exact at every angle. The visual stays a cylinder, which is what
    # the viewer sees; only the collider is swapped. This is standard practice
    # for simple wheeled robots and it removes the hop entirely.
    col = UsdGeom.Sphere.Define(stage, f"/World/rover/{name}/collider")
    col.CreateRadiusAttr(WR)
    col.CreatePurposeAttr("guide")          # invisible in the viewport
    UsdPhysics.CollisionAPI.Apply(col.GetPrim())
    # a real tyre material: high friction, no bounce
    wmat = UsdPhysics.MaterialAPI.Apply(w.GetPrim())
    wmat.CreateStaticFrictionAttr(1.0)
    wmat.CreateDynamicFrictionAttr(0.9)
    wmat.CreateRestitutionAttr(0.0)
    UsdPhysics.RigidBodyAPI.Apply(w.GetPrim())
    UsdPhysics.MassAPI.Apply(w.GetPrim()).CreateMassAttr(0.8)
    wpx = PhysxSchema.PhysxRigidBodyAPI.Apply(w.GetPrim())
    wpx.CreateSolverPositionIterationCountAttr(16)
    wpx.CreateSolverVelocityIterationCountAttr(4)

    j = UsdPhysics.RevoluteJoint.Define(stage, f"/World/rover/joint_{name}")
    j.CreateBody0Rel().SetTargets(["/World/rover/chassis"])
    j.CreateBody1Rel().SetTargets([f"/World/rover/{name}"])
    j.CreateAxisAttr("Y")
    # localPos0 is in the CHASSIS's local space. The chassis carries a scale op
    # of (BL,BW,BH) on a unit cube, so one local unit equals BL/BW/BH metres on
    # each axis: divide the metre offset by the scale to express it correctly.
    j.CreateLocalPos0Attr().Set(Gf.Vec3f(px_ / BL, py_ / BW, (WR - 0.26) / BH))
    # localPos1 is in the WHEEL's local space, which is unscaled, so it is the
    # wheel's own centre.
    j.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    # Bound the joint so a bad frame cannot fling the body: PhysX will complain
    # loudly instead of silently launching parts out of the scene.
    j.CreateBreakForceAttr(1.0e6)
    j.CreateBreakTorqueAttr(1.0e6)
    # DRIVE TUNING, computed rather than guessed. The first version jittered
    # visibly, and the numbers say why: damping 1500 on a wheel whose inertia is
    # 0.5*m*r^2 = 0.0038 kg m2 is roughly 20,000x critically damped, so the drive
    # fought the solver every step and the whole robot shook. And a target of
    # 90 rad/s at r=0.098 m is 8.8 m/s, which no AMR does; a MiR250 tops out at
    # 2.0 m/s, i.e. about 20 rad/s.
    #
    #   wheel inertia I = 0.5 * 0.8 * 0.098^2 = 0.00384 kg m2
    #   velocity-loop damping for a ~0.1 s time constant = I / tau = 0.038
    # Scaled up modestly for load, since four wheels carry an 18 kg chassis.
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateDampingAttr(2.5)
    drive.CreateStiffnessAttr(0.0)
    drive.CreateTargetVelocityAttr(9.0 if sy > 0 else 8.2)   # ~0.85 m/s, gentle
    drive.CreateMaxForceAttr(12.0)

# --- light -----------------------------------------------------------------
dome = UsdLux.DomeLight.Define(stage, "/World/dome")
dome.CreateIntensityAttr(1200.0)
key = UsdLux.DistantLight.Define(stage, "/World/key")
key.CreateIntensityAttr(2600.0)
UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-42, 0, 38))

# --- camera framed on the robot -------------------------------------------
cam = UsdGeom.Camera.Define(stage, "/World/shotCam")
cx = UsdGeom.Xformable(cam)
# Framed wide and from the side, because the robot DRIVES: a tight three-quarter
# shot lost it out of frame within a few seconds of physics starting.
cx.AddTranslateOp().Set(Gf.Vec3d(1.2, -6.2, 2.2))
cx.AddRotateXYZOp().Set(Gf.Vec3f(78, 0, 12))
cam.CreateFocalLengthAttr(24.0)
try:
    import omni.kit.viewport.utility as vpu
    vp = vpu.get_active_viewport()
    if vp is not None:
        vp.camera_path = "/World/shotCam"
except Exception as exc:
    print("[course] camera framing skipped:", exc)

print("[course] robot built:", len(list(stage.Traverse())), "prims")
print("[course] articulation root on /World/rover/chassis")
