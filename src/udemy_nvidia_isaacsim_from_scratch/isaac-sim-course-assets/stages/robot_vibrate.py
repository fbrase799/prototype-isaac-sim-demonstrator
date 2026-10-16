# 5.7 FAILURE: stiffness the timestep cannot resolve. Does it actually vibrate?
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import numpy as np, os
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf, UsdShade
import omni.usd
from isaacsim.core.utils.stage import clear_stage
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim
W=World(stage_units_in_meters=1.0)
CH_L,CH_W,CH_H=0.60,0.40,0.18; WH_R,WH_W=0.10,0.04; DECK=WH_R

def build(stiff, damp):
    clear_stage()
    st=omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(st,UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(st,1.0)
    w=UsdGeom.Xform.Define(st,"/World"); st.SetDefaultPrim(w.GetPrim())
    sc=UsdPhysics.Scene.Define(st,"/World/physicsScene")
    sc.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1)); sc.CreateGravityMagnitudeAttr(9.81)
    mat=UsdShade.Material.Define(st,"/World/m")
    pm=UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(0.9); pm.CreateDynamicFrictionAttr(0.85); pm.CreateRestitutionAttr(0.0)
    def bind(p): UsdShade.MaterialBindingAPI.Apply(p).Bind(mat,UsdShade.Tokens.weakerThanDescendants,"physics")
    g=UsdGeom.Cube.Define(st,"/World/ground"); g.CreateSizeAttr(1.0)
    gx=UsdGeom.Xformable(g); gx.AddTranslateOp().Set(Gf.Vec3d(0,0,-0.05)); gx.AddScaleOp().Set(Gf.Vec3f(20,20,0.1))
    UsdPhysics.CollisionAPI.Apply(g.GetPrim()); bind(g.GetPrim())
    rob=UsdGeom.Xform.Define(st,"/World/robot")
    UsdGeom.Xformable(rob).AddTranslateOp().Set(Gf.Vec3d(0,0,0))
    b=UsdGeom.Cube.Define(st,"/World/robot/chassis"); b.CreateSizeAttr(1.0)
    bx=UsdGeom.Xformable(b); bx.AddTranslateOp().Set(Gf.Vec3d(0,0,0.55)); bx.AddScaleOp().Set(Gf.Vec3f(CH_L,CH_W,CH_H))
    UsdPhysics.CollisionAPI.Apply(b.GetPrim()); UsdPhysics.RigidBodyAPI.Apply(b.GetPrim())
    UsdPhysics.MassAPI.Apply(b.GetPrim()).CreateMassAttr(12.0); bind(b.GetPrim())
    # PIN the chassis to the world. Left free it rolled and toppled at EVERY
    # stiffness, so the wheel speed measured the robot falling over rather than
    # the joint ringing, and the numbers showed no trend at all.
    anchor=UsdGeom.Cube.Define(st,"/World/anchor"); anchor.CreateSizeAttr(0.05)
    UsdGeom.Xformable(anchor).AddTranslateOp().Set(Gf.Vec3d(0,0,0.55))
    fix=UsdPhysics.FixedJoint.Define(st,"/World/robot/pin")
    fix.CreateBody0Rel().SetTargets([anchor.GetPrim().GetPath()])
    fix.CreateBody1Rel().SetTargets([b.GetPrim().GetPath()])
    fix.CreateLocalPos0Attr(Gf.Vec3f(0,0,0)); fix.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
    for name,side in (("wl",1),("wr",-1)):
        wp=UsdGeom.Cylinder.Define(st,f"/World/robot/{name}")
        wp.CreateRadiusAttr(WH_R); wp.CreateHeightAttr(WH_W); wp.CreateAxisAttr("Y")
        UsdGeom.Xformable(wp).AddTranslateOp().Set(Gf.Vec3d(-0.12,side*(CH_W/2+WH_W/2),0.55))
        UsdPhysics.CollisionAPI.Apply(wp.GetPrim()); UsdPhysics.RigidBodyAPI.Apply(wp.GetPrim())
        UsdPhysics.MassAPI.Apply(wp.GetPrim()).CreateMassAttr(0.8); bind(wp.GetPrim())
        j=UsdPhysics.RevoluteJoint.Define(st,f"/World/robot/j_{name}")
        j.CreateAxisAttr("Y")
        j.CreateBody0Rel().SetTargets([b.GetPrim().GetPath()])
        j.CreateBody1Rel().SetTargets([wp.GetPrim().GetPath()])
        j.CreateLocalPos0Attr(Gf.Vec3f(-0.12/CH_L, side*(CH_W/2+WH_W/2)/CH_W, 0.0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
        d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),"angular")
        d.CreateTypeAttr("force"); d.CreateStiffnessAttr(stiff); d.CreateDampingAttr(damp)
        d.CreateMaxForceAttr(1e6); d.CreateTargetPositionAttr(30.0)
    PhysxSchema.PhysxArticulationAPI.Apply(rob.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(rob.GetPrim())

def run(stiff,damp,steps=240):
    build(stiff,damp); W.reset()
    rp=RigidPrim("/World/robot/chassis"); rp.initialize()
    wl=RigidPrim("/World/robot/wl"); wl.initialize()
    zs=[]; vs=[]
    for _ in range(steps):
        W.step(render=False)
        zs.append(float(np.asarray(rp.get_world_poses()[0])[0][2]))  # chassis, pinned
        vs.append(float(np.linalg.norm(np.asarray(wl.get_angular_velocities())[0])))
    zs=np.array(zs[60:]); vs=np.array(vs[60:])
    return zs.std(), vs.max(), vs.std()

print("VIB>")
def show(l=""): print("VIB> "+l, flush=True)
show("a POSITION drive to 30 deg, damping 0, chassis pinned. does the joint ring?")
show(f"{'stiffness':>12} {'chassis z σ':>13} {'peak wheel ω':>14} {'ω σ':>11}")
show("-"*54)
for st_ in (1e2, 1e4, 1e6, 4e7):
    zsd, vmax, vsd = run(st_, 0.0)
    show(f"{st_:12.0e} {zsd:12.5f}m {vmax:13.2f} {vsd:10.2f}")
print("VIB_DONE", flush=True)
sim.close()
