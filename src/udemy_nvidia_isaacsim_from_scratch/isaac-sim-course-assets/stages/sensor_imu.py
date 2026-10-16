# 6.5: the IMU, and reading it honestly.
#
# The camera and the lidar answer questions about the world. An IMU does not:
# it reports what is happening to ITSELF, and the reading is only interpretable
# if you know what it is measuring and in which frame. Four things get measured:
#
#   1. A STATIONARY IMU DOES NOT READ ZERO. It reads +g on the axis opposite
#      gravity, because an accelerometer measures PROPER acceleration -- the
#      push of the floor holding it up -- not coordinate acceleration.
#
#   2. read_gravity=False GIVES A DIFFERENT ANSWER, and it is the one that
#      looks "right" while being the harder one to get from real hardware. Real
#      accelerometers cannot separate gravity from acceleration; that is an
#      estimator's job. Isaac will do it for you, which is a simulator luxury
#      worth naming out loud.
#
#   3. IN FREEFALL IT READS ZERO. The sensor goes quiet exactly when the robot
#      is accelerating hardest, because nothing is pushing on it any more. The
#      smoothing filter is measured here too: bigger filters buy quieter
#      numbers with delay.
#
#   4. THE READING IS IN THE SENSOR'S OWN FRAME. Roll the sensor and the same
#      physical situation moves between axes, exactly as g*sin / g*cos.
#
# TRAPS, all of which fail SILENTLY:
#
# - THE NAMESPACE. On Isaac 6.0 the IMU lives in
#   isaacsim.sensors.experimental.physics, and there is NO
#   isaacsim.sensors.physics module to import (only .examples, .nodes and .ui
#   exist). Tutorials written against 4.x import the latter and fail outright.
# - The sensor must be a CHILD of a rigid body. Parented to a bare Xform it
#   creates fine and then reports zeros forever.
# - WRITING A USD TRANSFORM DOES NOT MOVE A SIMULATING BODY. Once physics is
#   running PhysX owns the pose. Teleporting a body by setting its translate op
#   left it exactly where it was while the IMU read a steady 9.8067 for 25
#   steps: that looks like a broken sensor and is really a body that never
#   moved. Use RigidPrim.set_world_poses / set_velocities.
# - A RIGID BODY ADDED AFTER World.reset() IS NOT SIMULATED. Everything that
#   has to move is created before reset() here.
# - ORDER MATTERS, and this one cost the most time. The freefall test MUST run
#   before the roll test. The roll test makes the main body kinematic and
#   rewrites its xform ops every iteration; doing that first leaves the scene in
#   a state where the drop bodies no longer respond to set_world_poses at all --
#   they report z = 8.000 as though they had moved, while their IMUs sit at
#   9.8067 forever. The same code, run before the roll, works perfectly.
#   DESTRUCTIVE MEASUREMENTS GO LAST.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import math
import numpy as np
from pxr import UsdGeom, Gf, UsdPhysics, UsdLux
import omni.usd
from isaacsim.core.utils.stage import clear_stage
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim
from isaacsim.sensors.experimental.physics import IMU, IMUSensor

DT = 1.0 / 60.0
W = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=DT)


def show(l=""):
    print("IMU> " + l, flush=True)


clear_stage()
st = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
w = UsdGeom.Xform.Define(st, "/World")
st.SetDefaultPrim(w.GetPrim())
UsdLux.DomeLight.Define(st, "/World/dome").CreateIntensityAttr(800.0)

# ground, so the body has something to rest ON: the first measurement is
# entirely about the floor pushing back.
gp = UsdGeom.Cube.Define(st, "/World/ground")
gp.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(gp)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
gx.AddScaleOp().Set(Gf.Vec3f(40.0, 40.0, 1.0))
UsdPhysics.CollisionAPI.Apply(gp.GetPrim())


def rigid_box(path, pos, scale=0.4, mass=2.0):
    b = UsdGeom.Cube.Define(st, path)
    b.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(b)
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    x.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    UsdPhysics.CollisionAPI.Apply(b.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(b.GetPrim())
    UsdPhysics.MassAPI.Apply(b.GetPrim()).CreateMassAttr(mass)
    return b


# The resting body and the drop bodies are ALL created before reset().
body = rigid_box("/World/body", (0, 0, 0.25))
imu = IMUSensor(IMU.create("/World/body/imu", linear_acceleration_filter_size=1))

FILTERS = (1, 5, 20)
# SPACED BY INDEX, NOT BY FILTER SIZE. The first version placed them at
# x = 6*(N+1), which is 12, 36 and 126 m -- and the ground is only 40 m across,
# so the filter-5 and filter-20 bodies spawned over open space and were in
# freefall before the test began. Their "resting" reading was -0.0033, which
# the rest-check caught. A measurement scene has to be checked against its own
# geometry.
# x = 5, 10, 15: on the ground (which spans +-20) and clear of the resting body
# at x = 0, which the roll test later lifts to z = 2 and would otherwise be
# dropped on top of.
DROP_X = {N: 5.0 + 5.0 * i for i, N in enumerate(FILTERS)}   # 5, 10, 15
drops = {}
for N in FILTERS:
    dpath = f"/World/drop{N}"
    rigid_box(dpath, (DROP_X[N], 0, 0.25))
    drops[N] = (dpath, IMUSensor(IMU.create(
        f"{dpath}/imu", linear_acceleration_filter_size=N)))

W.reset()
# Let things settle: the first steps include the solver resolving the initial
# overlap with the ground, which is a real acceleration and not the resting
# case this lecture is about.
for _ in range(90):
    W.step(render=False)

f = imu.get_data()
if f is None or not np.isfinite(f["linear_acceleration"]).all():
    show("IMU NEVER RETURNED A VALID READING -- no table rather than zeros")
    print("IMU_DONE", flush=True)
    sim.close()
    raise SystemExit(1)

a = np.asarray(f["linear_acceleration"], dtype=float)
gyro = np.asarray(f["angular_velocity"], dtype=float)
if abs(a[2]) < 1.0:
    show(f"SUSPECT READING: |az| = {abs(a[2]):.4f}, expected about 9.81")
    show("the sensor is probably not parented to a rigid body")
    print("IMU_DONE", flush=True)
    sim.close()
    raise SystemExit(1)

# ---- 1. resting is not zero --------------------------------------------
show("a rigid body RESTING on the ground. Nothing is moving.")
show("")
show(f"   {'quantity':>22} {'x':>9} {'y':>9} {'z':>9}")
show("   " + "-" * 54)
show(f"   {'linear acceleration':>22} {a[0]:9.4f} {a[1]:9.4f} {a[2]:9.4f}")
show(f"   {'angular velocity':>22} {gyro[0]:9.4f} {gyro[1]:9.4f} {gyro[2]:9.4f}")
show("")
show(f"   it is NOT zero. z reads {a[2]:+.4f} m/s^2 while standing still,")
show("   because an accelerometer measures the FLOOR PUSHING BACK, not motion.")
show("")

# ---- 2. the same instant, with gravity removed -------------------------
f2 = imu.get_data(read_gravity=False)
a2 = np.asarray(f2["linear_acceleration"], dtype=float)
show("the SAME instant, read with read_gravity=False:")
show(f"   {'linear acceleration':>22} {a2[0]:9.4f} {a2[1]:9.4f} {a2[2]:9.4f}")
show("")
show(f"   {a[2]:+.4f} becomes {a2[2]:+.4f}: a simulator can subtract gravity")
show("   because it knows the orientation exactly. Real hardware cannot, which")
show("   is why every real robot runs an attitude estimator.")
show("")

# ---- 3. freefall, and what the smoothing filter costs -------------------
show("now drop it. In FREEFALL an accelerometer reads ZERO, not 9.81:")
show(f"   {'filter':>8} {'resting az':>12} {'az 1 step after release':>25} "
     f"{'steps to 10%':>13} {'ms @60Hz':>10}")
show("   " + "-" * 74)
# ALL THREE ARE DROPPED TOGETHER, in one shared timeline.
#
# Measuring them in sequence does not work: releasing one filter's body leaves
# it falling while the next is set up, and putting it back on the ground with
# set_world_poses mid-run does not take -- the later bodies stayed at -0.0033
# (freefall) no matter how many steps they were given to settle. Dropping every
# body from the same clean rest at the same instant sidesteps the problem
# entirely, and it is also a fairer comparison: identical input, three filters.
rps = {N: RigidPrim(drops[N][0]) for N in FILTERS}
rest = {}
for N in FILTERS:
    rest[N] = float(np.asarray(drops[N][1].get_data()["linear_acceleration"])[2])
bad = [N for N in FILTERS if abs(rest[N]) < 1.0]
if bad:
    show(f"   filters {bad} are NOT AT REST -- no table rather than a wrong one")
else:
    for N in FILTERS:
        rps[N].set_world_poses(positions=np.array([[DROP_X[N], 0.0, 8.0]]))
        rps[N].set_velocities(np.zeros((1, 6)))
    first, steps = {}, {}
    for k in range(1, 200):
        W.step(render=False)
        for N in FILTERS:
            z = float(np.asarray(
                drops[N][1].get_data()["linear_acceleration"])[2])
            if N not in first:
                first[N] = z
            if N not in steps and abs(z) <= abs(rest[N]) * 0.10:
                steps[N] = k
    for N in FILTERS:
        s = steps.get(N)
        ms = f"{s * DT * 1000:9.1f}ms" if s else "        --"
        sv = f"{s:13d}" if s else "           --"
        show(f"   {N:8d} {rest[N]:12.4f} {first[N]:25.4f} {sv} {ms}")
show("")
show("   The sensor goes QUIET exactly when the robot is accelerating hardest,")
show("   because nothing is pushing on it any more. A bigger filter buys")
show("   smoother numbers with delay, and that delay is time a controller")
show("   spends acting on a value the world has already left behind.")
show("")

# ---- 4. the reading lives in the SENSOR's frame -------------------------
# LAST, because it is destructive: it makes the body kinematic and rewrites its
# xform ops, which leaves the scene unusable for the drop test above.
show("roll the body, and the SAME situation moves between axes:")
# The predicted columns are +g*sin and +g*cos, NOT -g*sin. The first version
# printed the negative and it disagreed with every measured row: rolling about
# +x tips the gravity reaction onto +y, so both components are positive. The
# measurement was right and the prediction was wrong.
show(f"   {'roll':>8} {'ax':>9} {'ay':>9} {'az':>9} {'g*sin':>9} {'g*cos':>9} "
     f"{'|a|':>8}")
show("   " + "-" * 68)
UsdPhysics.RigidBodyAPI(body.GetPrim()).CreateKinematicEnabledAttr(True)
for deg in (0.0, 30.0, 60.0, 90.0):
    th = math.radians(deg)
    xf = UsdGeom.Xformable(body)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 2.0))
    xf.AddRotateXOp().Set(deg)
    xf.AddScaleOp().Set(Gf.Vec3f(0.4, 0.4, 0.4))
    for _ in range(20):
        W.step(render=False)
    ar = np.asarray(imu.get_data()["linear_acceleration"], dtype=float)
    show(f"   {deg:7.1f}d {ar[0]:9.4f} {ar[1]:9.4f} {ar[2]:9.4f} "
         f"{9.81 * math.sin(th):9.4f} {9.81 * math.cos(th):9.4f} "
         f"{float(np.linalg.norm(ar)):8.4f}")
show("")
show("   the magnitude never changes; only which axis carries it. If your IMU")
show("   'reads wrong', check the frame before you check the sensor.")

print("IMU_DONE", flush=True)
sim.close()
