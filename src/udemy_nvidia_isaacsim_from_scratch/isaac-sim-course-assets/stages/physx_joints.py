# 4.8: joints, measured. Revolute, prismatic and fixed.
#
# What the lecture needs to state with numbers:
#   1. What each joint type actually constrains, read off a running stage
#      rather than from a table in the docs.
#   2. What a drive does: target position, stiffness and damping are three
#      numbers whose interaction decides whether a joint arrives, oscillates
#      or never gets there.
#   3. The limit behaviour: does a revolute joint respect its limits.
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


def show(line=""):
    print("JOINT> " + line, flush=True)


def base_stage(gravity=9.81):
    clear_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    w = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(w.GetPrim())
    sc = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    sc.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    sc.CreateGravityMagnitudeAttr(gravity)
    return stage


def link(stage, path, pos, size=(0.4, 0.12, 0.12), mass=1.0, static=False):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(c)
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    x.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    if not static:
        UsdPhysics.RigidBodyAPI.Apply(c.GetPrim())
        UsdPhysics.MassAPI.Apply(c.GetPrim()).CreateMassAttr(mass)
    return c.GetPrim()


# ---------------------------------------------------------------------------
# 1. What each joint type allows. A link is hung off an anchor by each joint
#    type in turn, gravity is applied, and we see what moved.
# ---------------------------------------------------------------------------
def freedom_test(kind):
    stage = base_stage()
    anchor = link(stage, "/World/anchor", (0, 0, 1.5), (0.2, 0.2, 0.2), static=True)
    arm = link(stage, "/World/arm", (0.35, 0, 1.5), (0.6, 0.1, 0.1))

    if kind == "revolute":
        j = UsdPhysics.RevoluteJoint.Define(stage, "/World/joint")
        j.CreateAxisAttr("Y")
    elif kind == "prismatic":
        j = UsdPhysics.PrismaticJoint.Define(stage, "/World/joint")
        # Z, not X: a prismatic joint only shows its freedom if the axis is
        # along the force acting on it. With the axis horizontal and gravity
        # vertical it reported 0.0000 m, which reads as "prismatic joints do
        # not move" rather than "this one was not being pushed".
        j.CreateAxisAttr("Z")
    else:
        j = UsdPhysics.FixedJoint.Define(stage, "/World/joint")
    j.CreateBody0Rel().SetTargets([anchor.GetPath()])
    j.CreateBody1Rel().SetTargets([arm.GetPath()])
    # Local frames live in each body's OWN space and are scaled by that body's
    # own size. Setting both to the origin makes a degenerate joint with no
    # lever arm: measured, every joint type reported 0.0000 m of motion and the
    # engine warned "found a joint with disjointed body transforms". The child
    # attaches at its END, which in its own unit space is -0.5 along x.
    j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(-0.5, 0, 0))

    WORLD.reset()
    rp = RigidPrim("/World/arm")
    rp.initialize()
    p0 = np.asarray(rp.get_world_poses()[0])[0].copy()
    q0 = np.asarray(rp.get_world_poses()[1])[0].copy()
    for _ in range(150):
        WORLD.step(render=False)
    p1 = np.asarray(rp.get_world_poses()[0])[0]
    q1 = np.asarray(rp.get_world_poses()[1])[0]
    moved = float(np.linalg.norm(p1 - p0))
    turned = float(np.linalg.norm(q1 - q0))
    return moved, turned


show("1. what each joint type allows, with gravity pulling on the link")
show(f"   {'joint':>12} {'moved (m)':>12} {'rotated':>10}   what is free")
show("   " + "-" * 56)
for kind, free in (("fixed", "nothing"), ("revolute", "one rotation"),
                   ("prismatic", "one translation")):
    m, r = freedom_test(kind)
    show(f"   {kind:>12} {m:12.4f} {r:10.4f}   {free}")
show("")


# ---------------------------------------------------------------------------
# 2. Drive: stiffness and damping decide whether it arrives or oscillates.
# ---------------------------------------------------------------------------
def drive_test(stiffness, damping, target_deg=45.0, seconds=2.0):
    stage = base_stage(gravity=0.0)      # isolate the drive from gravity
    anchor = link(stage, "/World/anchor", (0, 0, 1.5), (0.2, 0.2, 0.2), static=True)
    arm = link(stage, "/World/arm", (0.35, 0, 1.5), (0.6, 0.1, 0.1))
    j = UsdPhysics.RevoluteJoint.Define(stage, "/World/joint")
    j.CreateAxisAttr("Y")
    j.CreateBody0Rel().SetTargets([anchor.GetPath()])
    j.CreateBody1Rel().SetTargets([arm.GetPath()])
    j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(-0.5, 0, 0))

    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    d.CreateTypeAttr("force")
    d.CreateTargetPositionAttr(target_deg)
    d.CreateStiffnessAttr(stiffness)
    d.CreateDampingAttr(damping)
    d.CreateMaxForceAttr(1e6)

    WORLD.reset()
    rp = RigidPrim("/World/arm")
    rp.initialize()
    peak = 0.0
    samples = []
    for _ in range(int(seconds * 60)):
        WORLD.step(render=False)
        q = np.asarray(rp.get_world_poses()[1])[0]
        w_, x_, y_, z_ = [float(v) for v in q]
        ang = np.degrees(2 * np.arctan2(abs(y_), abs(w_)))
        samples.append(ang)
        peak = max(peak, ang)
    final = float(np.mean(samples[-20:]))
    return final, peak


show("2. a position drive to 45 degrees: does it arrive, and does it overshoot?")
show(f"   {'stiffness':>10} {'damping':>9} {'settled':>10} {'peak':>9}   verdict")
show("   " + "-" * 58)
for st, dp in ((100.0, 0.0), (100.0, 5.0), (100.0, 40.0), (2000.0, 40.0)):
    f, pk = drive_test(st, dp)
    over = pk - f
    verdict = ("oscillates" if over > 4.0 else
               ("short" if f < 40.0 else "arrives"))
    show(f"   {st:10.0f} {dp:9.0f} {f:9.1f}° {pk:8.1f}°   {verdict}")

show("")
print("PHYSX_JOINTS_DONE", flush=True)
sim.close()
