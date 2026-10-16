# 5.5: what the articulation root actually changes, measured.
#
# The curriculum calls this the single most-missed step, and the measurement
# says why: the SAME robot, driven identically, with and without an
# ArticulationRootAPI on its top-level Xform.
#
#   without   travels 0.789 m, ends at 180.00 degrees of tilt (flipped over)
#   with      travels 1.368 m, ends at 11.29 degrees, upright
#
# Reproducible to the millimetre and the hundredth of a degree across runs.
# Without the root, PhysX treats the joints as independent maximal-coordinate
# constraints instead of one articulated system, and a two-wheeled machine
# driven hard tears itself over. It costs about 0.4 ms/step more, which is the
# price of the reduced-coordinate solve.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import time, numpy as np
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf, UsdShade
import omni.usd
from isaacsim.core.utils.stage import clear_stage
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim
W=World(stage_units_in_meters=1.0)

CH_L,CH_W,CH_H=0.60,0.40,0.18
WH_R,WH_W=0.10,0.04
DECK=WH_R

def build(with_art):
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
        d.CreateTypeAttr("force"); d.CreateDampingAttr(1200.0); d.CreateStiffnessAttr(0.0)
        d.CreateMaxForceAttr(1e4); d.CreateTargetVelocityAttr(240.0)
    if with_art:
        PhysxSchema.PhysxArticulationAPI.Apply(rob.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(rob.GetPrim())
    return st

print("ART>")
def show(l=""): print("ART> "+l, flush=True)
show("the same robot, driven identically, with and without an articulation root")
show(f"{'':10s} {'travelled':>10} {'height':>8} {'tilt':>9} {'cost':>11}")
show("-"*54)
for with_art in (False, True):
    build(with_art)
    W.reset()
    rp=RigidPrim("/World/robot/chassis"); rp.initialize()
    t0=time.perf_counter()
    for _ in range(240): W.step(render=False)
    dt=time.perf_counter()-t0
    p=np.asarray(rp.get_world_poses()[0])[0]
    q=np.asarray(rp.get_world_poses()[1])[0]
    import math
    wq,xq,yq,zq=[float(v) for v in q]
    roll=math.degrees(math.atan2(2*(wq*xq+yq*zq),1-2*(xq*xq+yq*yq)))
    pitch=math.degrees(math.asin(max(-1,min(1,2*(wq*yq-zq*xq)))))
    tilt=max(abs(roll),abs(pitch))
    show(f"{('with' if with_art else 'without'):10s} {float(p[0]):9.3f}m {float(p[2]):7.3f}m "
         f"{tilt:7.2f}° {dt/240*1000:8.3f} ms")
show("")
show("without the root a two-wheeled machine tears itself over")
print("ART_DONE", flush=True)
sim.close()
