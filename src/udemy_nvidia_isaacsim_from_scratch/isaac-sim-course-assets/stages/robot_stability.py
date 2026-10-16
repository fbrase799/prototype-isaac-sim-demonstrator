# 5.7: what actually destabilises a driven joint, measured.
#
# The curriculum planned this lecture as "the robot vibrates itself apart when
# stiffness and the timestep disagree". MEASURED, THAT IS NOT TRUE HERE, and
# the lecture says so rather than staging a failure that does not happen.
#
#   stiffness 5e1, no damping   peak 67.9 deg, settles 45.0, ringing 0.00
#   stiffness 5e2               peak 45.1
#   stiffness 5e4               peak 45.0
#   stiffness 5e6               peak 45.0, ringing 0.00, even with a 200 kg arm
#
# PhysX drives are solved implicitly, so raising the gain does not inject
# energy the way an explicit spring would. The instability the lecture was
# supposed to show is real in explicit integrators and in the CONTACT solver
# (4.7 measured a 12-box stack collapsing), but not in this drive.
#
# What DOES destabilise it is negative damping, which is energy injection by
# definition: at stiffness 5e2 with damping -20 the joint runs away to 359.9
# deg with a ringing sigma of 270.60.
#
# Three measurement bugs got in the way, all silent:
#   1. a free chassis rolled and toppled at EVERY stiffness, so the wheel speed
#      measured the robot falling over rather than the joint ringing
#   2. pinning the chassis with a fixed joint pinned the whole articulation and
#      every number went to exactly zero
#   3. a collider-only base gives the articulation ZERO degrees of freedom
#      (dof names: []) and every joint read raises IndexError. An articulation
#      link must be a rigid body; a fixed base is a body plus a fixed joint to
#      the world.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import numpy as np
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf
import omni.usd
from isaacsim.core.utils.stage import clear_stage
from isaacsim.core.api import World
W=World(stage_units_in_meters=1.0)

def build(stiff, damp):
    clear_stage()
    st=omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(st,UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(st,1.0)
    w=UsdGeom.Xform.Define(st,"/World"); st.SetDefaultPrim(w.GetPrim())
    sc=UsdPhysics.Scene.Define(st,"/World/physicsScene")
    sc.CreateGravityMagnitudeAttr(0.0)          # isolate the drive
    rob=UsdGeom.Xform.Define(st,"/World/rob")
    base=UsdGeom.Cube.Define(st,"/World/rob/base"); base.CreateSizeAttr(0.2)
    UsdGeom.Xformable(base).AddTranslateOp().Set(Gf.Vec3d(0,0,1.0))
    UsdPhysics.CollisionAPI.Apply(base.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
    UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr(10.0)
    root=UsdPhysics.FixedJoint.Define(st,"/World/rob/root")
    root.CreateBody1Rel().SetTargets([base.GetPrim().GetPath()])
    root.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
    arm=UsdGeom.Cube.Define(st,"/World/rob/arm"); arm.CreateSizeAttr(1.0)
    ax=UsdGeom.Xformable(arm); ax.AddTranslateOp().Set(Gf.Vec3d(0.3,0,1.0))
    ax.AddScaleOp().Set(Gf.Vec3f(0.6,0.1,0.1))
    UsdPhysics.CollisionAPI.Apply(arm.GetPrim()); UsdPhysics.RigidBodyAPI.Apply(arm.GetPrim())
    UsdPhysics.MassAPI.Apply(arm.GetPrim()).CreateMassAttr(globals().get("_MASS",1.0))
    j=UsdPhysics.RevoluteJoint.Define(st,"/World/rob/j")
    j.CreateAxisAttr("Y")
    j.CreateBody0Rel().SetTargets([base.GetPrim().GetPath()])
    j.CreateBody1Rel().SetTargets([arm.GetPrim().GetPath()])
    j.CreateLocalPos0Attr(Gf.Vec3f(0,0,0)); j.CreateLocalPos1Attr(Gf.Vec3f(-0.5,0,0))
    d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),"angular")
    d.CreateTypeAttr("force"); d.CreateStiffnessAttr(stiff); d.CreateDampingAttr(damp)
    d.CreateMaxForceAttr(1e7); d.CreateTargetPositionAttr(45.0)
    PhysxSchema.PhysxArticulationAPI.Apply(rob.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(rob.GetPrim())

from isaacsim.core.prims import Articulation
def run(stiff,damp,steps=180):
    build(stiff,damp); W.reset()
    art=Articulation("/World/rob"); art.initialize()
    if not globals().get("_reported"):
        globals()["_reported"]=True
        try:
            print("VIB>   dof names:", art.dof_names, flush=True)
            print("VIB>   num dof:", art.num_dof, flush=True)
        except Exception as e:
            print("VIB>   introspection failed:", type(e).__name__, str(e)[:60], flush=True)
    angs=[]
    for _ in range(steps):
        W.step(render=False)
        angs.append(float(np.degrees(np.asarray(art.get_joint_positions())[0][0])))
    a=np.array(angs)
    settle=a[90:]
    return a.max(), settle.std(), settle.mean()

print("VIB>")
def show(l=""): print("VIB> "+l, flush=True)
show("1. rising stiffness, no damping")
show(f"{'stiffness':>12} {'peak angle':>12} {'settled':>10} {'ringing σ':>11}")
show("-"*50)
for st_ in (50.0, 500.0, 5e4, 5e6):
    pk, sd, mn = run(st_, 0.0)
    show(f"{st_:12.0e} {pk:11.1f}° {mn:9.1f}° {sd:10.2f}°")
show("")
show("2. NEGATIVE damping, which is what actually injects energy")
show(f"{'stiffness':>12} {'damping':>10} {'peak angle':>12} {'ringing σ':>11}")
show("-"*52)
for st_, dp in ((500.0,-2.0),(500.0,-20.0),(5e4,-2.0)):
    try:
        pk, sd, mn = run(st_, dp)
        show(f"{st_:12.0e} {dp:10.1f} {pk:11.1f}° {sd:10.2f}°")
    except Exception as e:
        show(f"{st_:12.0e} {dp:10.1f} FAILED {type(e).__name__}")
show("")
show("3. a HEAVY link on a stiff drive: mass ratio, not stiffness alone")
for m in (1.0, 200.0):
    globals()["_MASS"]=m
    pk, sd, mn = run(5e6, 0.0)
    show(f"   arm mass {m:6.1f} kg  peak {pk:7.1f}°  ringing σ {sd:6.2f}°")
print("VIB_DONE", flush=True)
sim.close()
