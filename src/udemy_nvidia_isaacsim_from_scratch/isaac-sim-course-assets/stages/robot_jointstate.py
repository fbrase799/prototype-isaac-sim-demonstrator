# 5.8: reading joint state back out of the running articulation.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import numpy as np
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf, UsdShade
import omni.usd
from isaacsim.core.utils.stage import clear_stage
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation, RigidPrim
W=World(stage_units_in_meters=1.0)
CH_L,CH_W,CH_H=0.60,0.40,0.18; WH_R,WH_W=0.10,0.04; DECK=WH_R

def build(vl, vr):
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
    bx=UsdGeom.Xformable(b); bx.AddTranslateOp().Set(Gf.Vec3d(0,0,DECK+CH_H/2)); bx.AddScaleOp().Set(Gf.Vec3f(CH_L,CH_W,CH_H))
    UsdPhysics.CollisionAPI.Apply(b.GetPrim()); UsdPhysics.RigidBodyAPI.Apply(b.GetPrim())
    UsdPhysics.MassAPI.Apply(b.GetPrim()).CreateMassAttr(12.0); bind(b.GetPrim())
    c=UsdGeom.Sphere.Define(st,"/World/robot/caster"); c.CreateRadiusAttr(0.055)
    UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(0.335,0,0.055))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim()); UsdPhysics.RigidBodyAPI.Apply(c.GetPrim())
    UsdPhysics.MassAPI.Apply(c.GetPrim()).CreateMassAttr(0.3); bind(c.GetPrim())
    jc=UsdPhysics.FixedJoint.Define(st,"/World/robot/j_caster")
    jc.CreateBody0Rel().SetTargets([b.GetPrim().GetPath()])
    jc.CreateBody1Rel().SetTargets([c.GetPrim().GetPath()])
    jc.CreateLocalPos0Attr(Gf.Vec3f(0.335/CH_L,0,(0.055-DECK-CH_H/2)/CH_H))
    jc.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
    for name,side,tv in (("wheel_left",1,vl),("wheel_right",-1,vr)):
        wp=UsdGeom.Cylinder.Define(st,f"/World/robot/{name}")
        wp.CreateRadiusAttr(WH_R); wp.CreateHeightAttr(WH_W); wp.CreateAxisAttr("Y")
        UsdGeom.Xformable(wp).AddTranslateOp().Set(Gf.Vec3d(-0.12,side*(CH_W/2+WH_W/2),WH_R))
        UsdPhysics.CollisionAPI.Apply(wp.GetPrim()); UsdPhysics.RigidBodyAPI.Apply(wp.GetPrim())
        UsdPhysics.MassAPI.Apply(wp.GetPrim()).CreateMassAttr(0.8); bind(wp.GetPrim())
        j=UsdPhysics.RevoluteJoint.Define(st,f"/World/robot/j_{name}")
        j.CreateAxisAttr("Y")
        j.CreateBody0Rel().SetTargets([b.GetPrim().GetPath()])
        j.CreateBody1Rel().SetTargets([wp.GetPrim().GetPath()])
        j.CreateLocalPos0Attr(Gf.Vec3f(-0.12/CH_L, side*(CH_W/2+WH_W/2)/CH_W, (WH_R-DECK-CH_H/2)/CH_H))
        j.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
        d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),"angular")
        d.CreateTypeAttr("force"); d.CreateStiffnessAttr(0.0); d.CreateDampingAttr(600.0)
        d.CreateMaxForceAttr(1e4); d.CreateTargetVelocityAttr(tv)
    PhysxSchema.PhysxArticulationAPI.Apply(rob.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(rob.GetPrim())

def run(vl,vr,steps=240):
    build(vl,vr); W.reset()
    art=Articulation("/World/robot"); art.initialize()
    ch=RigidPrim("/World/robot/chassis"); ch.initialize()
    p0=np.asarray(ch.get_world_poses()[0])[0].copy()
    for _ in range(steps): W.step(render=False)
    p=np.asarray(ch.get_world_poses()[0])[0]
    q=np.asarray(ch.get_world_poses()[1])[0]
    import math
    wq,xq,yq,zq=[float(v) for v in q]
    yaw=math.degrees(math.atan2(2*(wq*zq+xq*yq),1-2*(yq*yq+zq*zq)))
    jp=np.degrees(np.asarray(art.get_joint_positions())[0])
    jv=np.degrees(np.asarray(art.get_joint_velocities())[0])
    return art.dof_names, float(p[0]-p0[0]), float(p[1]-p0[1]), yaw, jp, jv

print("JS>")
def show(l=""): print("JS> "+l, flush=True)
names,dx,dy,yaw,jp,jv = run(240.0,240.0)
show("the articulation reports its joints by name, in a fixed order:")
show("   "+str(list(names)))
show("")
show("differential drive: what the two wheel targets do")
show(f"{'left':>8} {'right':>8} {'dx':>8} {'dy':>8} {'yaw':>9}")
show("-"*46)
for vl,vr,lab in ((240.0,240.0,"straight"),(240.0,120.0,"curve"),(240.0,-240.0,"spin")):
    _n,dx,dy,yaw,jp,jv = run(vl,vr)
    show(f"{vl:8.0f} {vr:8.0f} {dx:7.3f}m {dy:7.3f}m {yaw:8.1f}°   {lab}")
show("")
show("and the joint velocities read back, commanded 240 °/s:")
_n,dx,dy,yaw,jp,jv = run(240.0,240.0)
for n,v in zip(_n,jv):
    show(f"   {n:16s} {v:8.1f} °/s")
print("JS_DONE", flush=True)
sim.close()
