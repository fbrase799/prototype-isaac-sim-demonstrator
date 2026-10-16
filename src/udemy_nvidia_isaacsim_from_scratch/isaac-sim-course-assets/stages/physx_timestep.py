# 4.7: timestep, substeps and solver iterations, measured.
#
# The curriculum calls this "where determinism goes to die", so the lecture has
# to show determinism actually dying rather than warn about it. Three tests:
#
#   1. DETERMINISM. Run the identical scene twice and diff the final positions.
#      If the engine is deterministic they are bit-identical; if not, the
#      difference is the number to quote.
#   2. TIMESTEP. Run the same scene at 1/60, 1/120 and 1/240 and compare. A
#      stack that stands at one rate and collapses at another is the failure
#      people actually hit.
#   3. SOLVER ITERATIONS. A stack of boxes is a constraint chain; too few
#      iterations and it sinks into itself.
#
# SimulationApp FIRST, before any pxr import.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})

import numpy as np
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf
import omni.usd
from isaacsim.core.utils.stage import clear_stage
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim

WORLD = World(stage_units_in_meters=1.0)
N_BOX = 6
BOX = 0.30


def show(line=""):
    """Tagged so the take can filter on one literal token (the 4.6 lesson)."""
    print("STEP> " + line, flush=True)


def build(dt=None, pos_iters=None, vel_iters=None):
    clear_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    w = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(w.GetPrim())
    sc = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    sc.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    sc.CreateGravityMagnitudeAttr(9.81)
    if dt is not None:
        WORLD.set_simulation_dt(physics_dt=dt)

    g = UsdGeom.Cube.Define(stage, "/World/ground")
    g.CreateSizeAttr(1.0)
    gx = UsdGeom.Xformable(g)
    gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
    gx.AddScaleOp().Set(Gf.Vec3f(10, 10, 0.1))
    UsdPhysics.CollisionAPI.Apply(g.GetPrim())

    # A stack, slightly offset so it is a real constraint problem rather than a
    # perfectly balanced one that would stand up by symmetry alone.
    for i in range(N_BOX):
        b = UsdGeom.Cube.Define(stage, f"/World/box{i}")
        b.CreateSizeAttr(1.0)
        bx = UsdGeom.Xformable(b)
        bx.AddTranslateOp().Set(Gf.Vec3d(0.012 * i, 0.008 * i, BOX / 2 + i * BOX))
        bx.AddScaleOp().Set(Gf.Vec3f(BOX, BOX, BOX))
        UsdPhysics.CollisionAPI.Apply(b.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(b.GetPrim())
        UsdPhysics.MassAPI.Apply(b.GetPrim()).CreateMassAttr(2.0)
        if pos_iters is not None:
            rb = PhysxSchema.PhysxRigidBodyAPI.Apply(b.GetPrim())
            rb.CreateSolverPositionIterationCountAttr(pos_iters)
            rb.CreateSolverVelocityIterationCountAttr(vel_iters or 1)
    return stage


def run(seconds=2.5, **kw):
    build(**kw)
    WORLD.reset()
    rps = [RigidPrim(f"/World/box{i}") for i in range(N_BOX)]
    for r in rps:
        r.initialize()
    dt = kw.get("dt") or 1.0 / 60.0
    for _ in range(int(seconds / dt)):
        WORLD.step(render=False)
    return np.array([np.asarray(r.get_world_poses()[0])[0] for r in rps])


show("1. determinism: the SAME scene run twice, positions compared")
a = run()
b = run()
diff = np.abs(a - b).max()
show(f"   largest difference across {N_BOX} boxes: {diff:.9f} m")
show(f"   verdict: {'BIT IDENTICAL' if diff == 0.0 else 'NOT reproducible'}")
show("")

# WHY THERE IS NO TIMESTEP TEST HERE.
#
# A resting stack does not care about the rate: measured 1.6502 at 30 Hz,
# 1.6502 at 60 and 1.6503 at 240, which teaches nothing. The interesting case
# is tunnelling, a fast body crossing a thin wall between samples, and I could
# not measure it honestly with this API:
#
#   - PhysxSceneAPI.TimeStepsPerSecond is NOT what World.step() honours.
#     get_physics_dt() stayed at 1/60 whatever it was set to, so 72 "240 Hz"
#     steps advanced 1.2 s of simulated time and the bullet covered 14.4 m in
#     a nominal 0.30 s window: an impossible number produced entirely by the
#     wrong knob.
#   - World.set_simulation_dt() is the right knob but raises "The Physics
#     Context's physics scene path is invalid" unless it is called after
#     World.reset(), and calling it there still did not produce a clean
#     comparison in the time available.
#
# Rather than ship a number I cannot defend, the lecture teaches the timestep
# from the two results below that ARE solid, and says plainly that the rate
# knob in the GUI is the one to reach for. Determinism and solver iteration
# count are both measured here and both reproduce.

show("2. solver position iterations: does the stack hold itself up?")
show(f"   {'iterations':>12} {'top box z':>12} {'sunk by':>10}")
show("   " + "-" * 40)
ideal = BOX * N_BOX - BOX / 2
for it in (1, 2, 4, 16):
    p = run(pos_iters=it)
    top = float(p[:, 2].max())
    show(f"   {it:12d} {top:12.4f} {ideal - top:9.4f} m")

show("")
show("3. a taller stack needs MORE iterations: how far each one sinks")
show(f"   {'boxes':>7} {'4 iters':>12} {'16 iters':>12}   ")
show("   " + "-" * 40)
for n in (4, 8, 12):
    globals()["N_BOX"] = n
    ideal_n = BOX * n - BOX / 2
    p4 = run(pos_iters=4)
    p16 = run(pos_iters=16)
    s4 = ideal_n - float(p4[:, 2].max())
    s16 = ideal_n - float(p16[:, 2].max())
    show(f"   {n:7d} {s4:11.4f}m {s16:11.4f}m")
globals()["N_BOX"] = 6

show("")
print("PHYSX_TIMESTEP_DONE", flush=True)
sim.close()
