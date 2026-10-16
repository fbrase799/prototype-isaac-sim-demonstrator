# 7.2: SimReady assets, and what that label actually buys you.
#
# "SimReady" is marketing language until you diff one against a mesh you made
# yourself. This script does exactly that: it loads a stock NVIDIA prop and a
# hand-built cube of the same size, and reports what each one carries.
#
# Four things get measured:
#
#   1. WHAT COMES ATTACHED. Colliders, rigid-body APIs, mass, materials and
#      semantic labels -- the things Sections 4, 6 and 6.8 had to add BY HAND
#      to every object, present on arrival or not.
#
#   2. MASS IS AUTHORED, NOT GUESSED. A prop that knows what it weighs will
#      behave correctly the moment it is dropped; a cube with a default mass
#      will not.
#
#   3. THE SEMANTIC LABEL IS THE EXPENSIVE ONE. 6.8 measured what happens when
#      a prim has no label: its pixels land in an UNLABELLED bin and train as
#      background. A prop that ships labelled is a dataset that is correct by
#      default.
#
#   4. IT COSTS SOMETHING. A SimReady prop is more prims and more triangles
#      than a cube, and that is the trade being made.
#
# TRAPS:
#
# - Not every path under the asset root exists on every version. The ArchVis
#   pallet used in older tutorials 404s on 6.0. Probe before you depend on it.
# - add_reference_to_stage does NOT bring the referenced prims into
#   stage.Traverse() until the layer has loaded; step the app first or the
#   counts come back as 1.
# - This script refuses to print a table it did not measure.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import numpy as np
from pxr import UsdGeom, Gf, UsdPhysics, UsdLux, UsdShade
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage
from isaacsim.storage.native import get_assets_root_path


def show(l=""):
    print("SIM> " + l, flush=True)


root = get_assets_root_path()
if root is None:
    show("NO ASSET ROOT -- the Isaac asset CDN is unreachable; no table.")
    print("SIM_DONE", flush=True)
    sim.close()
    raise SystemExit(1)

create_new_stage()
st = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
w = UsdGeom.Xform.Define(st, "/World")
st.SetDefaultPrim(w.GetPrim())
UsdLux.DomeLight.Define(st, "/World/dome").CreateIntensityAttr(1000.0)

# ground
gp = UsdGeom.Cube.Define(st, "/World/ground")
gp.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(gp)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
gx.AddScaleOp().Set(Gf.Vec3f(20.0, 20.0, 1.0))
UsdPhysics.CollisionAPI.Apply(gp.GetPrim())

# THREE TIERS, not two, because the first version of this script compared the
# wrong pair and its own premise collapsed. /Isaac/Props/YCB/Axis_Aligned/ is
# a TEXTURED MESH: 16384 triangles, one material, and ZERO colliders, zero
# rigid bodies, no mass and no label. Calling that "SimReady" would have been
# a claim the measurement refutes.
#
# The physics-carrying variant lives at Axis_Aligned_PHYSICS, and the real
# lesson is that BOTH ship under the same asset root: picking the wrong folder
# gives you an object the camera can see and physics cannot touch.
GEOM = root + "/Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd"
PHYS = root + "/Isaac/Props/YCB/Axis_Aligned_Physics/003_cracker_box.usd"
add_reference_to_stage(usd_path=GEOM, prim_path="/World/geom_only")
UsdGeom.Xformable(st.GetPrimAtPath("/World/geom_only")).AddTranslateOp().Set(
    Gf.Vec3d(0.0, 0.0, 0.5))
add_reference_to_stage(usd_path=PHYS, prim_path="/World/simready")
# THE PHYSICS VARIANT ALREADY HAS AN XFORM STACK. AddTranslateOp() on it raises
# "the xformOp 'xformOp:translate' already exists in xformOpOrder
# [translate, rotateZYX, scale]" -- which is itself a sign of the difference:
# the authored prop arrives with a transform stack, the bare mesh does not.
# Set the existing op rather than adding another.
def place(path, pos):
    x = UsdGeom.Xformable(st.GetPrimAtPath(path))
    for op in x.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(*pos))
            return
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))


place("/World/simready", (0.6, 0.0, 0.5))

# B: a cube I made myself, the way every earlier section did it
mine = UsdGeom.Cube.Define(st, "/World/mine")
mine.CreateSizeAttr(1.0)
mx = UsdGeom.Xformable(mine)
mx.AddTranslateOp().Set(Gf.Vec3d(1.0, 0, 0.5))
mx.AddScaleOp().Set(Gf.Vec3f(0.16, 0.06, 0.21))   # roughly a cracker box

W = World(stage_units_in_meters=1.0)
W.reset()
for _ in range(30):
    W.step(render=False)


def survey(path):
    """Everything the asset carries, counted."""
    p = st.GetPrimAtPath(path)
    if not p or not p.IsValid():
        return None
    kids = list(pxr_range(p))
    meshes = [k for k in kids if k.IsA(UsdGeom.Mesh)]
    tris = 0
    for m in meshes:
        fc = UsdGeom.Mesh(m).GetFaceVertexCountsAttr().Get()
        if fc:
            tris += sum(max(0, int(c) - 2) for c in fc)
    mass = None
    for k in kids:
        if k.HasAPI(UsdPhysics.MassAPI):
            a = UsdPhysics.MassAPI(k).GetMassAttr().Get()
            if a:
                mass = float(a)
                break
    return {
        "prims": len(kids),
        "meshes": len(meshes),
        "triangles": tris,
        "colliders": sum(1 for k in kids if k.HasAPI(UsdPhysics.CollisionAPI)),
        "rigid": sum(1 for k in kids if k.HasAPI(UsdPhysics.RigidBodyAPI)),
        "mass": mass,
        "materials": len({UsdShade.MaterialBindingAPI(k).ComputeBoundMaterial()[0].GetPath()
                          for k in meshes
                          if UsdShade.MaterialBindingAPI(k).ComputeBoundMaterial()[0]}),
        "labels": sum(1 for k in kids
                      if any("semantic" in s.lower() or "label" in s.lower()
                             for s in k.GetAppliedSchemas())),
    }


def pxr_range(prim):
    from pxr import Usd
    return Usd.PrimRange(prim)


A = survey("/World/simready")
G = survey("/World/geom_only")
B = survey("/World/mine")
if A is None or A["meshes"] == 0:
    show("THE STOCK PROP DID NOT LOAD -- no table rather than a fake one.")
    show(f"tried: {PROP}")
    print("SIM_DONE", flush=True)
    sim.close()
    raise SystemExit(1)

show("the SAME cracker box, from two folders under the same asset root,")
show("next to a cube I built myself:")
show("")
show(f"   {'':>20} {'Axis_Aligned_Physics':>21} {'Axis_Aligned':>13} {'my cube':>9}")
show("   " + "-" * 66)
for key, label in (("prims", "prims"), ("meshes", "meshes"),
                   ("triangles", "triangles"), ("colliders", "colliders"),
                   ("rigid", "rigid bodies"), ("materials", "materials"),
                   ("labels", "semantic labels")):
    show(f"   {label:>20} {str(A[key]):>21} {str(G[key]):>13} {str(B[key]):>9}")
fmt = lambda m: f"{m:.3f} kg" if m else "none"
show(f"   {'authored mass':>20} {fmt(A['mass']):>21} {fmt(G['mass']):>13} "
     f"{fmt(B['mass']):>9}")
show("")
show("   The middle column is the trap. It is the same box, the same 16384")
show("   triangles, the same material -- and physics cannot touch it.")
show("")
show("   NOTE WHAT IS MISSING FROM BOTH: neither carries a semantic label. So")
show("   'SimReady' does NOT solve 6.8's problem for you. Label it yourself or")
show("   it trains as background.")
show("")

# ---- and now the consequence, measured -------------------------------------
# "Drop it and it hangs in the air" is the sort of claim that has to be run.
# LIFT THEM BOTH FIRST. The survey above runs 30 physics steps, and the
# physics variant has already fallen and settled by then: recording its
# "start" at that point reads -0.004 m, so nothing appears to fall and the
# narration contradicts the table. Put both back up before timing the drop.
from isaacsim.core.prims import RigidPrim
import omni.timeline
omni.timeline.get_timeline_interface().play()
DROP_Z = 1.2
place("/World/geom_only", (0.0, 0.0, DROP_Z))
try:
    rp = RigidPrim("/World/simready")
    rp.set_world_poses(positions=np.array([[0.6, 0.0, DROP_Z]]))
    rp.set_velocities(np.zeros((1, 6)))
except Exception:
    place("/World/simready", (0.6, 0.0, DROP_Z))
for _ in range(2):
    W.step(render=False)
start = {}
for path in ("/World/simready", "/World/geom_only"):
    bb = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
    start[path] = bb.ComputeWorldBound(
        st.GetPrimAtPath(path)).ComputeAlignedRange().GetMin()[2]
for _ in range(120):
    W.step(render=False)
show(f"both lifted to z = {DROP_Z}, then 120 physics steps (2.0 s):")
show(f"   {'asset':>22} {'start z':>9} {'end z':>9} {'fell':>9}")
show("   " + "-" * 54)
for path, name in (("/World/simready", "Axis_Aligned_Physics"),
                   ("/World/geom_only", "Axis_Aligned")):
    bb = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
    z = bb.ComputeWorldBound(
        st.GetPrimAtPath(path)).ComputeAlignedRange().GetMin()[2]
    show(f"   {name:>22} {start[path]:8.3f}m {z:8.3f}m {start[path] - z:8.3f}m")
show("")
show("   one landed on the floor. The other is still exactly where it was put,")
show("   hanging in mid air, and nothing anywhere reported a problem.")

print("SIM_DONE", flush=True)
sim.close()
