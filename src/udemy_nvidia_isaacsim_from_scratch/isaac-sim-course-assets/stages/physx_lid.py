# 4.2: the invisible lid, shown in the viewport.
#
# Two identical channels side by side with two identical balls dropped into
# them. The only difference is physics:approximation. The left one is
# convexDecomposition and the ball falls IN; the right one is convexHull and
# the ball stops dead at the top of the walls, resting on nothing you can see.
#
# The measurement script (physx_approx.py) proves this numerically: ball rest z
# is 0.301 with decomposition and 0.980 with a hull, identical to the millimetre
# on every run. This stage is the same fact made watchable.
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf, UsdLux

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)

# a ground plane so the scene has a floor line to read heights against
ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(ground)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
gx.AddScaleOp().Set(Gf.Vec3f(14, 14, 0.1))
ground.CreateDisplayColorAttr([(0.62, 0.64, 0.66)])
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())


def channel(path, ox, colour):
    """The same U-shaped channel as the measurement script."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    pts, counts, idx = [], [], []

    def box(cx, cy, cz, sx, sy, sz):
        base = len(pts)
        for dx in (-1, 1):
            for dy in (-1, 1):
                for dz in (-1, 1):
                    pts.append(Gf.Vec3f(cx + dx * sx / 2, cy + dy * sy / 2,
                                        cz + dz * sz / 2))
        for f in [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1),
                  (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]:
            counts.append(4)
            idx.extend([base + i for i in f])

    box(0.0, 0.0, 0.05, 1.6, 1.0, 0.10)
    box(-0.75, 0.0, 0.40, 0.10, 1.0, 0.80)
    box(0.75, 0.0, 0.40, 0.10, 1.0, 0.80)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateDisplayColorAttr([colour])
    UsdGeom.Xformable(mesh).AddTranslateOp().Set(Gf.Vec3d(ox, 0, 0))
    return mesh


CASES = [
    ("good", -1.35, "convexDecomposition", (0.35, 0.62, 0.05)),
    ("lid",   1.35, "convexHull",          (0.90, 0.29, 0.30)),
]
BALL_START_Z = 1.85

for name, ox, approx, colour in CASES:
    m = channel(f"/World/{name}_channel", ox, colour)
    prim = m.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approx)

    b = UsdGeom.Sphere.Define(stage, f"/World/{name}_ball")
    b.CreateRadiusAttr(0.18)
    UsdGeom.Xformable(b).AddTranslateOp().Set(Gf.Vec3d(ox, 0, BALL_START_Z))
    b.CreateDisplayColorAttr([(0.16, 0.17, 0.19)])
    UsdPhysics.CollisionAPI.Apply(b.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(b.GetPrim())

UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(1100.0)

# Framing solved the same way as 4.1: content spans x = -2.2 to 2.2 (channels
# are 1.6 wide centred at +/-1.35), so half-width 2.2 plus margin = 2.5, and
# d = half_width / tan(hfov/2) = 5.72 m at 24 mm. Use 6.0 for headroom.
cam = UsdGeom.Camera.Define(stage, "/World/shot_cam")
cam.CreateFocalLengthAttr(24.0)
# 6.0 m at 24 mm clipped the right channel's outer wall: the outer edge is at
# x=2.15 and with margin the shot must hold 2.60, which needs 5.95 m at 24 mm
# with NO slack. Pulled back to 6.7 m so both channels sit inside the frame.
m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(0.0, -6.7, 1.70),
                            Gf.Vec3d(0.0, 0.0, 0.62),
                            Gf.Vec3d(0, 0, 1)).GetInverse()
UsdGeom.Xformable(cam).AddTransformOp().Set(m)

try:
    import omni.kit.viewport.utility as vp_util
    vp = vp_util.get_active_viewport()
    if vp is not None:
        vp.camera_path = "/World/shot_cam"
except Exception as exc:                       # noqa: BLE001
    print(f"  viewport camera not set: {exc}", flush=True)

print("PHYSX_LID_STAGE_READY", flush=True)

# Loop the drop, for the same reason as 4.1: one drop is over in a second and
# the take needs a minute of usable footage. Wall clock, not a payload dt.
RESET_EVERY = 5.0
try:
    import time as _time
    import omni.timeline
    import omni.kit.app
    from pxr import UsdPhysics as _UP

    omni.timeline.get_timeline_interface().play()
    print("PHYSX_LID_PLAYING", flush=True)

    _balls = [(stage.GetPrimAtPath(f"/World/{n}_ball"), ox) for n, ox, _a, _c in CASES]
    _state = {"last": _time.monotonic()}

    def _reset(e):
        now = _time.monotonic()
        if now - _state["last"] < RESET_EVERY:
            return
        _state["last"] = now
        for prim, ox in _balls:
            for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(ox, 0, BALL_START_Z))
            rb = _UP.RigidBodyAPI(prim)
            rb.GetVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
            rb.GetAngularVelocityAttr().Set(Gf.Vec3f(0, 0, 0))

    _sub = (omni.kit.app.get_app().get_update_event_stream()
            .create_subscription_to_pop(_reset, name="physx_lid_loop"))
    print("PHYSX_LID_LOOPING", flush=True)
except Exception as exc:                       # noqa: BLE001
    print(f"  could not start the timeline: {exc}", flush=True)
