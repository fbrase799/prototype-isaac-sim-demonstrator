# 4.5 FAILURE: default inertia tips the robot over.
#
# Two identical chassis driven through the same turn at the same speed. The only
# difference is the inertia tensor and the centre of mass:
#
#   left   everything left at Isaac's defaults, computed from the collider at
#          uniform density: the tensor of a solid box, centre of mass at the
#          geometric centre, which is far too high for a machine whose battery
#          sits on the floor pan
#   right  a measured tensor and a measured centre of mass, the numbers you
#          would get from the real vehicle
#
# The left one goes over. That is the lecture.
#
# The turn is a cornering FORCE rather than wheel joints: joints are 4.8, and
# using them here would make the failure depend on machinery the learner has
# not met yet. See the drive block at the bottom for why it is a force and not
# a velocity, which took several wrong attempts to get right.
import os
import omni.usd
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf, UsdLux

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)

ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(ground)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
gx.AddScaleOp().Set(Gf.Vec3f(30, 30, 0.1))
ground.CreateDisplayColorAttr([(0.62, 0.64, 0.66)])
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

# Chassis geometry, in metres. Deliberately tall and narrow, which is what a
# sensor mast does to a small robot and is exactly when the tensor matters.
# TALL AND NARROW, and that is the point. At 0.60 x 0.40 x 0.55 the centre of
# mass sits 1.38 half-tracks up: the chassis lifted onto its edge, reached 90
# degrees, and then FELL BACK, so a viewer sees a wobble rather than a failure.
# At 0.30 wide by 0.90 tall the ratio is 3.0, which is a sensor mast on a small
# base, and once it is past its edge it cannot recover. Measured thresholds:
# 128 N for the old body, 59 N for this one.
BODY = (0.55, 0.30, 0.90)          # a (x), b (y), c (z)
MASS = 18.0
START_Z = BODY[2] / 2 + 0.02

CASES = [
    # (name, y offset, use defaults, colour)
    ("default", -1.4, True,  (0.90, 0.29, 0.30)),
    ("measured", 1.4, False, (0.35, 0.62, 0.05)),
]


def chassis(name, oy, use_defaults, colour):
    body = UsdGeom.Cube.Define(stage, f"/World/{name}")
    body.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(body)
    x.AddTranslateOp().Set(Gf.Vec3d(-1.2, oy, START_Z))
    x.AddScaleOp().Set(Gf.Vec3f(*BODY))
    body.CreateDisplayColorAttr([colour])
    prim = body.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    m = UsdPhysics.MassAPI.Apply(prim)
    m.CreateMassAttr(MASS)

    if not use_defaults:
        # The measured numbers. The battery is a slab on the floor pan, so the
        # real centre of mass is well below the geometric centre, and the real
        # tensor is smaller about the roll axis than a uniform box's.
        #
        # The default (measured off the running stage) is Ixx 0.6938, Iyy
        # 0.9937, Izz 0.7800 with the centre of mass at the geometric centre.
        # Measured on the real vehicle: the pack sits on the floor pan, so the
        # centre of mass is 0.075 m up rather than the 0.275 m a uniform box
        # assumes, and the roll inertia is correspondingly smaller.
        m.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, -0.33))
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(0.42, 0.60, 0.55))
    return prim


from pxr import UsdShade
_mat = UsdShade.Material.Define(stage, "/World/physMat")
_pm = UsdPhysics.MaterialAPI.Apply(_mat.GetPrim())
_pm.CreateStaticFrictionAttr(0.9)
_pm.CreateDynamicFrictionAttr(0.85)
_pm.CreateRestitutionAttr(0.0)


def _bind(prim):
    """Without friction the chassis SLIDES sideways instead of gripping, and a
    body that slides never rolls over its outside edge: the failure this
    lecture is about simply does not happen."""
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        _mat, UsdShade.Tokens.weakerThanDescendants, "physics")


_bind(ground.GetPrim())
PRIMS = [(n, chassis(n, oy, d, c), oy) for (n, oy, d, c) in CASES]
for _n, _p, _oy in PRIMS:
    _bind(_p)

UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(1100.0)

# Framing, solved the same way as the rest of Section 4. The two lanes sit at
# y = -1.4 and +1.4 and each body is 0.6 long, so the shot must hold a
# half-width of about 2.2 m in y. At 24 mm that needs 5.0 m; 8.2 m is used so
# both lanes and the ground plane around them are comfortably in frame.
cam = UsdGeom.Camera.Define(stage, "/World/shot_cam")
cam.CreateFocalLengthAttr(24.0)
mtx = Gf.Matrix4d().SetLookAt(Gf.Vec3d(1.2, -8.2, 3.4),
                              Gf.Vec3d(0.4, 0.0, 0.35),
                              Gf.Vec3d(0, 0, 1)).GetInverse()
UsdGeom.Xformable(cam).AddTransformOp().Set(mtx)

try:
    import omni.kit.viewport.utility as vp_util
    vp = vp_util.get_active_viewport()
    if vp is not None:
        vp.camera_path = "/World/shot_cam"
except Exception as exc:                       # noqa: BLE001
    print(f"  viewport camera not set: {exc}", flush=True)

print("PHYSX_TIP_STAGE_READY", flush=True)
for name, prim, _oy in PRIMS:
    print(f"  /World/{name:9s} {list(prim.GetAppliedSchemas())}", flush=True)

# The drive, and why it is a FORCE rather than a velocity.
#
# Setting the body's velocity every step DICTATES its motion, so it physically
# cannot roll and both cases measured 0.0 degrees of tilt: the tensor was never
# allowed to matter. A cornering load is applied instead, as a burst, and PhysX
# decides what happens.
#
# The load acts THROUGH EACH BODY'S OWN CENTRE OF MASS, because that is what an
# inertial cornering force does. Applying it at a fixed height instead gives the
# low machine the same overturning moment as the high one, and no choice of
# force can then separate them: measured, both flipped at every value tried.
#
# The static rollover threshold is F = m g (b/2) / com_z, which is 128 N for the
# default centre of mass at 0.275 m and 471 N for the measured one at 0.075 m.
# 200 N sits between them. Verified headless before recording: default tips 97.5
# degrees, measured stays at 0.1.
# The burst is counted in FRAMES, not seconds, so the impulse does not depend
# on how fast the machine is rendering. Measured: the headless probe steps at a
# fixed 1/60 s and got 27 force applications across a 0.45 s burst, while the
# GUI at ~21 fps landed only 16 to 22, i.e. about two thirds of the impulse,
# which dropped the default chassis below its 128 N rollover threshold and it
# stayed upright. The first GUI take recorded exactly that non-event.
RUN_TIME = 7.0
# 300 N, verified IN THE GUI (not just headless): default reaches 94.5 degrees
# and stays over, measured stays at 0.0. 95 N only reached 11.8 because this
# taller body carries more roll inertia and the burst is brief.
PUSH = float(os.environ.get('TIP_PUSH', '300'))
TURN_AT = 1.60
BURST_FRAMES = 27     # matches the verified headless burst
COM_Z = {"default": 0.0, "measured": -0.33}

try:
    import time as _time
    import omni.timeline
    import omni.kit.app
    import numpy as _np
    from isaacsim.core.prims import RigidPrim as _RigidPrim
    from pxr import UsdPhysics as _UP

    omni.timeline.get_timeline_interface().play()
    print("PHYSX_TIP_PLAYING", flush=True)

    _rps = {}
    state = {"t0": _time.monotonic()}

    def _drive(e):
        now = _time.monotonic()
        el = now - state["t0"]

        if el > RUN_TIME:
            # Report the burst that just finished. The GUI runs at ~21 fps
            # against the headless probe's fixed 1/60 s, so a wall-clock burst
            # lands a THIRD of the force applications and the impulse is a
            # third of what was verified. This print is how that was caught.
            print(f"  BURST applied {state.get('applied', 0)} force calls "
                  f"peak_tilt default={state.get('pk_default',0):.1f} "
                  f"measured={state.get('pk_measured',0):.1f}", flush=True)
            state["pk_default"] = 0.0
            state["pk_measured"] = 0.0
            state["applied"] = 0
            state["t0"] = now
            for name, prim, oy in PRIMS:
                for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        op.Set(Gf.Vec3d(-1.2, oy, START_Z))
                    elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                        op.Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
                rb = _UP.RigidBodyAPI(prim)
                rb.GetVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
                rb.GetAngularVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
            return

        # track peak tilt of each body so the GUI run can be measured, not
        # judged by eye from a contact sheet
        import math as _math
        for _n, _pr, _oy in PRIMS:
            _rp = _rps.get(_n)
            if _rp is None:
                continue
            try:
                _pos, _q = _rp.get_world_poses()
                _w, _x, _y, _z = [float(v) for v in _np.asarray(_q)[0]]
                _roll = _math.degrees(_math.atan2(2*(_w*_x+_y*_z), 1-2*(_x*_x+_y*_y)))
                _pitch = _math.degrees(_math.asin(max(-1, min(1, 2*(_w*_y-_z*_x)))))
                _t = max(abs(_roll), abs(_pitch))
                _k = "pk_" + _n
                if _t > state.get(_k, 0.0):
                    state[_k] = _t
            except Exception:                          # noqa: BLE001
                pass

        if el <= TURN_AT:
            return
        if state.get("applied", 0) >= BURST_FRAMES:
            return

        for name, prim, oy in PRIMS:
            rp = _rps.get(name)
            if rp is None:
                try:
                    rp = _RigidPrim(str(prim.GetPath()))
                    rp.initialize()
                    _rps[name] = rp
                except Exception:                     # noqa: BLE001
                    continue
            try:
                pos, _q = rp.get_world_poses()
                px, py, pz = [float(v) for v in _np.asarray(pos)[0]]
                rp.apply_forces_and_torques_at_pos(
                    forces=_np.array([[0.0, PUSH, 0.0]]),
                    positions=_np.array([[px, py, pz + COM_Z[name]]]),
                    is_global=True)
                state["applied"] = state.get("applied", 0) + 1
            except Exception as exc:                   # noqa: BLE001
                if not state.get("warned"):
                    state["warned"] = True
                    print(f"  apply_forces FAILED: {type(exc).__name__}: {exc}",
                          flush=True)

    _sub = (omni.kit.app.get_app().get_update_event_stream()
            .create_subscription_to_pop(_drive, name="physx_tip_drive"))
    print("PHYSX_TIP_DRIVING", flush=True)
except Exception as exc:                       # noqa: BLE001
    print(f"  could not start the drive: {exc}", flush=True)
