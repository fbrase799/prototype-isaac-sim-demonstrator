# 4.2: what shape the collider ACTUALLY is, and what each approximation costs.
#
# The curriculum requires measured cost, not vibes. This script builds the same
# concave mesh (a U-shaped channel, chosen because its concavity is the whole
# point: a hull fills the channel in and a cup stops being a cup) and reports,
# per approximation:
#
#   - the approximation token USD actually stores
#   - whether the concavity SURVIVES, tested physically by dropping a ball into
#     the channel and seeing whether it comes to rest inside or on top
#   - the simulation step cost, timed over N steps on this machine
#
# Timing is the reason this file exists rather than a slide: "convexHull is
# cheaper" is repeated everywhere and the interesting question is how much, on
# hardware the learner might have.
#
# SimulationApp FIRST, before any pxr import (see 3.8/3.9/4.1).
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})

import os
import time
import statistics
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, UsdLux


def build_channel(stage, path):
    """A U-shaped channel: floor plus two walls, as ONE mesh.

    Concave on purpose. A convex approximation cannot represent the gap, so a
    ball dropped in lands on an invisible lid at the top of the walls, which is
    exactly the bug this lecture is about.
    """
    mesh = UsdGeom.Mesh.Define(stage, path)
    # three slabs welded into one mesh: base, left wall, right wall
    pts, counts, idx = [], [], []

    def box(cx, cy, cz, sx, sy, sz):
        base = len(pts)
        for dx in (-1, 1):
            for dy in (-1, 1):
                for dz in (-1, 1):
                    pts.append(Gf.Vec3f(cx + dx * sx / 2,
                                        cy + dy * sy / 2,
                                        cz + dz * sz / 2))
        # 6 quads per box, indices into the 8 corners just appended
        faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1),
                 (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
        for f in faces:
            counts.append(4)
            idx.extend([base + i for i in f])

    box(0.0, 0.0, 0.05, 1.6, 1.0, 0.10)      # base
    box(-0.75, 0.0, 0.40, 0.10, 1.0, 0.80)   # left wall
    box(0.75, 0.0, 0.40, 0.10, 1.0, 0.80)    # right wall

    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateDisplayColorAttr([(0.62, 0.64, 0.66)])
    return mesh


APPROXIMATIONS = ["none", "convexHull", "convexDecomposition", "boundingCube"]
STEPS = 240          # how many physics steps each variant is timed over
# ONE ball on ONE channel could not measure anything: repeated three times, the
# within-variant spread reached 0.589 ms/step while the whole between-variant
# range was 0.412 ms, so the ordering shuffled run to run. The scene has to be
# big enough that collision work dominates the step. GRID x GRID channels, each
# with a stack of balls.
GRID = int(os.environ.get("APPROX_GRID", "6"))
BALLS_PER = int(os.environ.get("APPROX_BALLS", "3"))
REPEATS = int(os.environ.get("APPROX_REPEATS", "3"))
BALL_START_Z = 1.20  # above the walls, so it must fall INTO the channel


def measure(approx, world):
    """Build the case into the CONTEXT's stage and step it.

    Not an in-memory stage: omni.usd's attach_stage_with_callback is a pybind
    builtin whose binding rejected a Usd.Stage here (TypeError: incompatible
    function arguments), and World is a singleton that cannot be reconstructed
    per case (it warns "World is already initialized" and ignores the new
    parameters). Building into the context stage and clearing it between cases
    avoids both.
    """
    import omni.usd
    from isaacsim.core.utils.stage import clear_stage

    clear_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    w_prim = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(w_prim.GetPrim())

    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)

    stored = None
    for gx in range(GRID):
        for gy in range(GRID):
            ox, oy = gx * 2.2, gy * 1.6
            mesh = build_channel(stage, f"/World/channel_{gx}_{gy}")
            UsdGeom.Xformable(mesh).AddTranslateOp().Set(Gf.Vec3d(ox, oy, 0))
            prim = mesh.GetPrim()
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approx)
            if stored is None:
                stored = prim.GetAttribute("physics:approximation").Get()
            for b in range(BALLS_PER):
                ball = UsdGeom.Sphere.Define(stage, f"/World/ball_{gx}_{gy}_{b}")
                ball.CreateRadiusAttr(0.18)
                UsdGeom.Xformable(ball).AddTranslateOp().Set(
                    Gf.Vec3d(ox, oy, BALL_START_Z + b * 0.5))
                UsdPhysics.CollisionAPI.Apply(ball.GetPrim())
                UsdPhysics.RigidBodyAPI.Apply(ball.GetPrim())

    world.reset()
    t0 = time.perf_counter()
    for _ in range(STEPS):
        world.step(render=False)
    dt = time.perf_counter() - t0

    z = None
    for op in UsdGeom.Xformable(stage.GetPrimAtPath("/World/ball_0_0_0")).GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            z = op.Get()[2]
    return stored, dt / STEPS * 1000.0, z


from isaacsim.core.api import World
WORLD = World(stage_units_in_meters=1.0)

print("")
print("PHYSX_APPROX")
print(f"scene: {GRID}x{GRID} channels, {BALLS_PER} balls each = "
      f"{GRID*GRID} colliders + {GRID*GRID*BALLS_PER} rigid bodies, "
      f"median of {REPEATS} runs")
print(f"{'approximation':<22} {'stored token':<22} {'ms/step':>9} {'spread':>8} {'ball rest z':>12}  concavity")
print("-" * 96)

rows = []
for a in APPROXIMATIONS:
    try:
        samples = []
        for _ in range(REPEATS):
            stored, ms, z = measure(a, WORLD)
            samples.append(ms)
        ms = statistics.median(samples)
        spread = max(samples) - min(samples)
        # The ball radius is 0.18 and the channel floor top is at z=0.10, so a
        # ball that fell INTO the channel rests near 0.28. One that sat on an
        # invisible convex lid rests near the wall top, 0.80 + 0.18 = 0.98.
        kept = "kept" if (z is not None and z < 0.55) else "LOST"
        print(f"{a:<22} {str(stored):<22} {ms:9.3f} {spread:8.3f} {z:12.3f}  {kept}")
        rows.append((a, stored, ms, spread, z, kept))
    except Exception as exc:                       # noqa: BLE001
        print(f"{a:<22} FAILED: {type(exc).__name__}: {str(exc)[:44]}")

# RATIOS, not absolute milliseconds. Two runs of this same scene measured the
# mesh case at 7.386 and at 3.271 ms/step depending on what else the machine was
# doing, but mesh/hull came out 2.12x and 1.73x: the ORDERING and the rough
# factor are what reproduce, so those are what the lecture quotes.
if rows:
    base = next((r[2] for r in rows if r[0] == "convexHull"), None)
    if base:
        print("")
        print("relative to convexHull (the cheap one):")
        for a, _stored, ms, _sp, _z, kept in rows:
            print(f"  {a:<22} {ms/base:5.2f}x   concavity {kept}")

print("")
print("PHYSX_APPROX_DONE", flush=True)
sim.close()
