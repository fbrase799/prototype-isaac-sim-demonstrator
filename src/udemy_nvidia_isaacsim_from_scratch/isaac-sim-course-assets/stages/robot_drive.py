# 5.6: what drive stiffness and damping actually do to a WHEEL, and the units.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import numpy as np, math, os
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf, UsdShade
import omni.usd
from isaacsim.core.utils.stage import clear_stage
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim
W=World(stage_units_in_meters=1.0)
CH_L,CH_W,CH_H=0.60,0.40,0.18; WH_R,WH_W=0.10,0.04; DECK=WH_R

def build(stiff, damp, target_vel):
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
    for name,side in (("wl",1),("wr",-1)):
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
        d.CreateTypeAttr("force"); d.CreateStiffnessAttr(stiff); d.CreateDampingAttr(damp)
        d.CreateMaxForceAttr(1e4); d.CreateTargetVelocityAttr(target_vel)
    PhysxSchema.PhysxArticulationAPI.Apply(rob.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(rob.GetPrim())

def run(stiff,damp,tv,steps=300):
    """Measure the STEADY-STATE speed, not an instantaneous sample.

    Reading get_linear_velocities() at one arbitrary step picks up acceleration
    and wheel slip and produced numbers that moved around with no pattern
    (0.514 m/s at a 120 deg/s target against 0.289 at 240). Averaging the
    displacement over the SECOND HALF of the run, after it has settled, gives a
    speed that matches the wheel arithmetic.
    """
    build(stiff,damp,tv); W.reset()
    rp=RigidPrim("/World/robot/chassis"); rp.initialize()
    for _ in range(steps//2): W.step(render=False)     # let it reach speed
    pa=np.asarray(rp.get_world_poses()[0])[0].copy()
    for _ in range(steps//2): W.step(render=False)
    pb=np.asarray(rp.get_world_poses()[0])[0]
    dt=(steps//2)/60.0
    v=float(abs(pb[0]-pa[0]))/dt
    return float(pb[0]), v

print("DRV>")
def show(l=""): print("DRV> "+l, flush=True)
show("target velocity is in DEGREES per second for an angular drive")
show(f"{'target':>10} {'implied m/s':>12} {'measured m/s':>13} {'travelled':>11}")
show("-"*50)
for tv in (120.0, 240.0, 480.0):
    dx,v = run(0.0, 1200.0, tv)
    implied = math.radians(tv)*WH_R
    show(f"{tv:9.0f}° {implied:11.3f} {v:13.3f} {dx:10.3f}m")
show("")
show("stiffness vs damping on a VELOCITY drive")
show(f"{'stiffness':>10} {'damping':>9} {'measured m/s':>13} {'travelled':>11}")
show("-"*48)
for st_,dp in ((0.0,1200.0),(0.0,200.0),(0.0,20.0),(500.0,1200.0)):
    dx,v = run(st_,dp,240.0)
    show(f"{st_:10.0f} {dp:9.0f} {v:13.3f} {dx:10.3f}m")
print("DRV_DONE", flush=True)
sim.close()
