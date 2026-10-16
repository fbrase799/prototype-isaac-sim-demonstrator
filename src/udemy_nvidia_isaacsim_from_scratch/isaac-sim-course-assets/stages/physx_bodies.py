# 4.1: rigid bodies and colliders are DIFFERENT THINGS, built so the difference
# is visible rather than asserted.
#
# The lecture's claim is that RigidBodyAPI (does it move?) and CollisionAPI
# (does it stop things?) are independent, and that three of the four
# combinations are useful. Four cubes, one per combination, dropped over a
# ground plane:
#
#   both      falls and lands            a normal dynamic prop
#   body only falls THROUGH the ground   the classic "my robot fell through
#                                        the floor" bug
#   coll only never moves, blocks things a static wall
#   neither   never moves, blocks nothing  a purely visual prop
#
# The ground has CollisionAPI and no RigidBodyAPI, which is what makes it a
# floor rather than a falling slab: that is the same distinction, applied to
# the thing the learner already takes for granted.
import omni.usd
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf, UsdLux

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

# The physics scene itself. Without it nothing simulates at all, and the
# symptom is "I pressed play and nothing moved", which reads as a broken body.
scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr(9.81)

# ground: collider, NOT a rigid body
ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(ground)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
gx.AddScaleOp().Set(Gf.Vec3f(9, 9, 0.1))
ground.CreateDisplayColorAttr([(0.62, 0.64, 0.66)])
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

# Colours are chosen to READ AGAINST A LIGHT VIEWPORT. The first version used
# 0.75 and 0.55 greys for the two static cubes and they were nearly invisible
# in the recorded frame against the pale ground and background: a cube the
# lecture is comparing has to be visible in the same shot as the others.
CASES = [
    ("both",      True,  True,  (0.35, 0.62, 0.05)),   # green: works
    ("body_only", True,  False, (0.90, 0.29, 0.30)),   # red: falls through
    ("coll_only", False, True,  (0.22, 0.42, 0.75)),   # blue: static wall
    ("neither",   False, False, (0.34, 0.36, 0.40)),   # dark grey: inert
]

for i, (name, rigid, coll, colour) in enumerate(CASES):
    c = UsdGeom.Cube.Define(stage, f"/World/{name}")
    c.CreateSizeAttr(0.5)
    cx = UsdGeom.Xformable(c)
    cx.AddTranslateOp().Set(Gf.Vec3d(-2.0 + i * 1.35, 0, 1.6))
    c.CreateDisplayColorAttr([colour])
    if rigid:
        UsdPhysics.RigidBodyAPI.Apply(c.GetPrim())
    if coll:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())

UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(1100.0)

# A label over each cube, so the four cases are readable in the recorded frame
# without narration having to say "second from the left". UsdGeom has no text
# prim, so these are thin coloured posts: green for an applied API, grey for a
# missing one, body on the left post and collider on the right.
for i, (name, rigid, coll, _c) in enumerate(CASES):
    x = -2.0 + i * 1.35
    for j, (on, dx) in enumerate(((rigid, -0.16), (coll, 0.16))):
        post = UsdGeom.Cube.Define(stage, f"/World/tag_{name}_{j}")
        post.CreateSizeAttr(1.0)
        px = UsdGeom.Xformable(post)
        px.AddTranslateOp().Set(Gf.Vec3d(x + dx, 0, 2.45))
        px.AddScaleOp().Set(Gf.Vec3f(0.12, 0.12, 0.34))
        post.CreateDisplayColorAttr([(0.35, 0.62, 0.05) if on else (0.72, 0.73, 0.75)])

# The shot. Authored as its own camera prim and pointed at with a look-at
# matrix, because writing /OmniverseKit_Persp's xformOps does not stick and
# hand-solved Euler angles put the camera inside the subject (see 3.4/3.7).
# Framing is SOLVED, not guessed. Two takes were wasted guessing: 28 mm at
# 9.4 m made the cubes unreadably small, and 35 mm at 6.2 m cropped the blue
# and grey cubes clean out of the frame, so only two of the four cases the
# lecture compares were visible.
#
# The content spans x = -2.0 to 2.05 with a half-width of 0.25 per cube, so the
# shot must hold |x| <= 2.65 including margin. With the 35 mm-equivalent
# horizontal aperture of 20.955 mm, the distance needed is
#   d = half_width / tan(hfov/2),  hfov = 2*atan(aperture/(2*focal))
# which gives 6.07 m at 24 mm, 7.08 at 28 and 8.85 at 35. 24 mm at 6.6 m keeps
# every cube inside the frame with room to spare, and is as tight as the four
# cases allow.
cam = UsdGeom.Camera.Define(stage, "/World/shot_cam")
cam.CreateFocalLengthAttr(24.0)
eye = Gf.Vec3d(0.0, -6.6, 1.9)
target = Gf.Vec3d(0.0, 0.0, 0.80)
m = Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse()
UsdGeom.Xformable(cam).AddTransformOp().Set(m)

try:
    import omni.kit.viewport.utility as vp_util
    vp = vp_util.get_active_viewport()
    if vp is not None:
        vp.camera_path = "/World/shot_cam"
except Exception as exc:                      # noqa: BLE001
    print(f"  viewport camera not set: {exc}", flush=True)

# Read the APIs back OUT of the stage rather than printing what we just asked
# for: the lecture quotes these names on screen, and the schema is the
# authority, not this script's intent.
print("PHYSX_BODIES_STAGE_READY", flush=True)
for name in [c[0] for c in CASES] + ["ground"]:
    prim = stage.GetPrimAtPath(f"/World/{name}")
    schemas = prim.GetAppliedSchemas()
    print(f"  /World/{name:10s} {list(schemas)}", flush=True)

# Start the simulation, so the take records the DROP rather than a still life.
# Timeline play is what makes rigid bodies fall; without it the four cases look
# identical and the whole lecture is unwatchable.
#
# The drop itself lasts about a second, and the take needs ~70 s of usable
# footage. A single drop would leave ~69 s of three cubes sitting still and one
# gone, which is a frozen frame in everything but name and would be flagged by
# check_frozen (correctly). So the drop is LOOPED: every RESET_EVERY seconds the
# cubes are lifted back to their start heights and released again, which also
# means any offset the assemble step picks lands on a drop in progress.
RESET_EVERY = 4.0
START_Z = 1.6

try:
    import omni.timeline
    import omni.kit.app
    from pxr import UsdPhysics as _UsdPhysics

    _timeline = omni.timeline.get_timeline_interface()
    _timeline.play()
    print("PHYSX_BODIES_PLAYING", flush=True)

    # Only the RIGID bodies are reset: the two static cubes never moved, so
    # rewriting their transforms would do nothing except make the loop look
    # like it is doing more than it is.
    _movers = [
        (stage.GetPrimAtPath(f"/World/{name}"), -2.0 + i * 1.35)
        for i, (name, is_rigid, _coll, _colour) in enumerate(CASES) if is_rigid
    ]

    import time as _time
    _state = {"last": _time.monotonic()}

    def _reset_drop(e):
        """Lift the dynamic cubes back to the top and re-release them.

        Timed on the WALL CLOCK, not on a dt from the event payload: a probe of
        the update event stream showed the payload has no dt to read (and the
        subscription may not fire at all under a headless app), so accumulating
        e.payload["dt"] would silently never reach the reset threshold and the
        take would record one drop and 69 s of stillness.

        Velocities are ZEROED on reset: a body repositioned while it still
        carries its fall velocity keeps accelerating from where it left off, so
        each loop would be faster than the last and the four cases would stop
        being comparable.
        """
        now = _time.monotonic()
        if now - _state["last"] < RESET_EVERY:
            return
        _state["last"] = now
        for prim, x in _movers:
            xf = UsdGeom.Xformable(prim)
            for op in xf.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(x, 0, START_Z))
            rb = _UsdPhysics.RigidBodyAPI(prim)
            rb.GetVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
            rb.GetAngularVelocityAttr().Set(Gf.Vec3f(0, 0, 0))

    _sub = (omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(_reset_drop, name="physx_bodies_loop"))
    print("PHYSX_BODIES_LOOPING", flush=True)
except Exception as exc:                      # noqa: BLE001
    print(f"  could not start the timeline: {exc}", flush=True)
