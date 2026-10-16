# Offscreen render of NVIDIA's Nova Carter, driving, to a PNG sequence.
#
# WHY OFFSCREEN. Three attempts to frame the interactive viewport on a chosen
# camera all reverted to Kit's own startup framing: switching vp.camera_path
# immediately, switching it on a deferred update tick, and moving the default
# perspective camera in the scene script. The viewport owns its camera during
# startup and fights anything set around it.
#
# Rendering offscreen removes the argument entirely. A render product is bound to
# a camera prim explicitly, nothing else can retarget it, and the framing is
# exactly what this file says it is. It also renders at whatever resolution we
# ask for regardless of the window, and it does not need the GUI to be visible,
# so it cannot be spoiled by a window manager or a stray raise.
#
# Run inside the container as:
#   /isaac-sim/kit/kit <app.kit> --exec "/workspace/stages/render_carter.py"
#
# Writes /workspace/out/carter/frame_####.png, which the host then encodes.

import os
import carb
import omni.kit.app
import omni.usd
import omni.replicator.core as rep
from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, PhysxSchema, Gf

OUT = "/workspace/out/carter"
FRAMES = int(os.environ.get("CARTER_FRAMES", "420"))     # 14 s at 30 fps
W, H = 1600, 900

CARTER = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
          "/Assets/Isaac/4.5/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd")

os.makedirs(OUT, exist_ok=True)

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

# --- physics --------------------------------------------------------------
scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr().Set(9.81)
psx = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
psx.CreateSolverTypeAttr("TGS")
psx.CreateTimeStepsPerSecondAttr(120)

# --- a floor worth photographing -------------------------------------------
# A seamless light floor, matching the course's studio look so the real footage
# and the animated scenes sit in the same world.
ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(ground)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
gx.AddScaleOp().Set(Gf.Vec3f(80.0, 80.0, 0.1))
ground.CreateDisplayColorAttr([(0.55, 0.56, 0.58)])
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
gmat = UsdPhysics.MaterialAPI.Apply(ground.GetPrim())
gmat.CreateStaticFrictionAttr(1.0)
gmat.CreateDynamicFrictionAttr(0.9)
gmat.CreateRestitutionAttr(0.0)

# --- the robot, referenced from NVIDIA's library ---------------------------
rover = UsdGeom.Xform.Define(stage, "/World/carter")
rover.GetPrim().GetReferences().AddReference(CARTER)
rx = UsdGeom.Xformable(rover)


def xform_op(xformable, kind, name_hint):
    """Get an existing xformOp of this kind, or add one.

    A REFERENCED prim usually arrives with its own xformOpOrder already
    populated. The Nova Carter reference carries [translate, orient, scale], so
    calling AddTranslateOp() on it throws 'The xformOp already exists' and the
    whole render script dies before writing a single frame. Reuse what is there.
    """
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == kind:
            return op
    return {
        UsdGeom.XformOp.TypeTranslate: xformable.AddTranslateOp,
        UsdGeom.XformOp.TypeRotateZ: xformable.AddRotateZOp,
        UsdGeom.XformOp.TypeRotateXYZ: xformable.AddRotateXYZOp,
    }[kind]()


t_op = xform_op(rx, UsdGeom.XformOp.TypeTranslate, "t")
t_op.Set(Gf.Vec3d(0, 0, 0))

# --- lighting: a key, a fill and a soft dome -------------------------------
dome = UsdLux.DomeLight.Define(stage, "/World/dome")
dome.CreateIntensityAttr(900.0)
key = UsdLux.DistantLight.Define(stage, "/World/key")
key.CreateIntensityAttr(120.0)
key.CreateAngleAttr(1.6)                       # a soft-edged shadow
UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-42, 0, 40))
fill = UsdLux.DistantLight.Define(stage, "/World/fill")
fill.CreateIntensityAttr(40.0)
fill.CreateAngleAttr(3.0)
UsdGeom.Xformable(fill).AddRotateXYZOp().Set(Gf.Vec3f(-24, 0, -128))

# --- THE CAMERA, framed explicitly and bound to a render product -----------
# This is the whole point of rendering offscreen: the render product takes a
# camera PRIM PATH, so nothing can retarget it later.
cam = UsdGeom.Camera.Define(stage, "/World/renderCam")
cx = UsdGeom.Xformable(cam)
cam_t = xform_op(cx, UsdGeom.XformOp.TypeTranslate, "t")
cam_r = xform_op(cx, UsdGeom.XformOp.TypeRotateXYZ, "r")
cam.CreateFocalLengthAttr(32.0)
cam.CreateFocusDistanceAttr(2.4)
cam.CreateFStopAttr(8.0)
cam.CreateExposureAttr(-1.2)          # pull the whole frame down off the clip                       # a little depth of field

# REAL-TIME RTX, not path tracing.
#
# The first working render produced soft, washed-out frames. That was not the
# lighting: it was CONVERGENCE. Path tracing accumulates samples per frame, and
# rep.orchestrator.step(rt_subframes=1) gives it four, which is nowhere near
# enough, so every frame is an under-sampled blur. Raising subframes fixes it but
# costs roughly linear time, and a 400 frame clip becomes an overnight job.
#
# RaytracedLighting is the real-time RTX path: rasterised with ray-traced
# lighting, converged in ONE frame by construction. It is also exactly what the
# interactive viewport was using when it rendered Carter sharply, so we already
# have visual proof it looks right on this hardware.
_st = carb.settings.get_settings()
_st.set("/rtx/rendermode", "RaytracedLighting")

# DLSS OFF, and no upscaling. THIS is what made the frames soft, not sampling.
#
# The blur was uniform across the WHOLE frame, including the static floor, which
# is not what under-convergence looks like: sampling noise concentrates on
# surfaces and shadows, not on flat empty ground. What is uniform is UPSCALING.
# gpu_lib.sh sets --/rtx/post/dlss/execMode=1 and pins the renderer to 1280x720
# for the interactive viewport, which is a sensible VRAM mitigation on an 80 W
# 4050 while a window is being screen-grabbed. Offscreen has no such constraint:
# the render product owns its own resolution, so render at native and skip DLSS.
_st.set("/rtx/post/dlss/execMode", 0)
_st.set("/rtx/post/aa/op", 3)              # DLAA-style AA without upscaling
_st.set("/rtx/post/scaling/enabled", False)
_st.set("/app/renderer/resolution/width", W)
_st.set("/app/renderer/resolution/height", H)
_st.set("/rtx/directLighting/sampledLighting/enabled", True)

rp = rep.create.render_product("/World/renderCam", (W, H))
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=OUT, rgb=True)
writer.attach([rp])

app = omni.kit.app.get_app()


import math


def robot_bounds():
    """The robot's real world-space extent, so framing is measured not guessed.

    The first attempt hand-computed an orbit and the robot was never in shot:
    every frame rendered empty floor. Deriving the distance and the aim point
    from the actual bounding box removes the guesswork.
    """
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    b = cache.ComputeWorldBound(stage.GetPrimAtPath("/World/carter"))
    r = b.ComputeAlignedRange()
    if r.IsEmpty():
        return Gf.Vec3d(0, 0, 0.25), 0.9
    mn, mx = r.GetMin(), r.GetMax()
    centre = Gf.Vec3d((mn[0]+mx[0])/2, (mn[1]+mx[1])/2, (mn[2]+mx[2])/2)
    size = max(mx[0]-mn[0], mx[1]-mn[1], mx[2]-mn[2])
    return centre, max(0.35, size)


def look_at(eye, target):
    """Euler XYZ that points a USD camera (which looks down -Z) at target."""
    d = Gf.Vec3d(target[0]-eye[0], target[1]-eye[1], target[2]-eye[2])
    n = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]) or 1.0
    d = Gf.Vec3d(d[0]/n, d[1]/n, d[2]/n)
    pitch = math.degrees(math.acos(max(-1.0, min(1.0, -d[2]))))
    yaw = math.degrees(math.atan2(d[1], d[0])) - 90.0
    return Gf.Vec3f(pitch, 0.0, yaw)


CENTRE, SIZE = robot_bounds()
print(f"[render] robot centre {CENTRE} size {SIZE:.2f} m")
# frame so the machine fills a comfortable share of a 40mm shot
DIST = max(2.6, SIZE * 4.2)


def camera_at(i):
    """A slow arc around the robot, aimed by look-at rather than by hand."""
    u = i / max(1, FRAMES - 1)
    ang = math.radians(-118.0 + 44.0 * u)
    tx = 0.60 * u * 1.9                       # the robot travels with the shot
    target = Gf.Vec3d(CENTRE[0] + tx, CENTRE[1], CENTRE[2])
    eye = Gf.Vec3d(target[0] + DIST*math.cos(ang),
                   target[1] + DIST*math.sin(ang),
                   CENTRE[2] + SIZE*0.85)
    cam_t.Set(eye)
    cam_r.Set(look_at(eye, target))
    return tx


# let the reference resolve and the shaders warm before the first frame, or the
# opening frames render grey and untextured
for _ in range(180):
    app.update()

print("[render] warm, starting", FRAMES, "frames")

timeline = None
try:
    import omni.timeline
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
except Exception as exc:
    print("[render] timeline unavailable:", exc)

# Drive the frames by pumping Kit itself. rep.orchestrator.step() is a
# STANDALONE-workflow call and raises inside --exec, so it cannot be used here.
for i in range(FRAMES):
    tx = camera_at(i)
    t_op.Set(Gf.Vec3d(0.60 * (i / max(1, FRAMES - 1)) * 1.9, 0.0, 0.0))
    # a few updates per output frame: one to apply the transforms, the rest to
    # let the renderer settle before the writer picks the frame up
    for _ in range(4):
        app.update()
    if i % 30 == 0:
        print(f"[render] frame {i}/{FRAMES}")

print("[render] done ->", OUT)
try:
    omni.kit.app.get_app().post_quit()
except Exception:
    pass
