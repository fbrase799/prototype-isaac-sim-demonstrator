# 5.9: import a URDF and read back what Isaac ACTUALLY made of it.
#
# The forum's explicit request is "how do I get my own robot in", so this
# imports a real URDF and reads the result rather than describing the dialog.
# Measured on 6.0.1:
#
#   12 prims, with PhysicsArticulationRootAPI placed on base_link for you
#   both `continuous` joints became PhysicsRevoluteJoint, with
#     PhysicsDriveAPI:angular and PhysicsJointStateAPI:angular already applied
#   every link got TWO geoms: a visual and a `_1` carrying PhysicsCollisionAPI
#   base_link 12.0 kg, wheel_left 0.8 kg, and wheel_right NOT AUTHORED
#
# That last line is the point. The URDF deliberately omits the inertial on
# wheel_right, and the importer passes the omission straight through: no error,
# no warning in the result, just a link with no mass attribute.
#
# Two API facts worth keeping:
#   - there is NO _urdf module in 6.0.1. `from ... import _urdf` raises
#     ImportError; the public API is URDFImporter / URDFImporterConfig.
#   - import_urdf() returns the path of the USD FILE it wrote, not a prim in
#     the open stage. Traversing the current stage after importing finds
#     nothing at all, which looks exactly like a failed import.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import os, numpy as np
from pxr import Usd, UsdGeom, UsdPhysics
import omni.usd
from isaacsim.core.utils.stage import clear_stage

URDF = "/workspace/out/probe_bot.urdf"
os.makedirs("/workspace/out", exist_ok=True)
# A minimal but honest URDF: base, two wheels on continuous joints, and a
# deliberately MISSING inertial on one link, which is the most common real
# defect in URDFs found in the wild.
open(URDF,"w").write('''<?xml version="1.0"?>
<robot name="probe_bot">
  <link name="base_link">
    <visual><geometry><box size="0.6 0.4 0.18"/></geometry></visual>
    <collision><geometry><box size="0.6 0.4 0.18"/></geometry></collision>
    <inertial><mass value="12.0"/>
      <inertia ixx="0.19" ixy="0" ixz="0" iyy="0.39" iyz="0" izz="0.52"/>
    </inertial>
  </link>
  <link name="wheel_left">
    <visual><geometry><cylinder radius="0.1" length="0.04"/></geometry></visual>
    <collision><geometry><cylinder radius="0.1" length="0.04"/></geometry></collision>
    <inertial><mass value="0.8"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.004"/>
    </inertial>
  </link>
  <link name="wheel_right">
    <visual><geometry><cylinder radius="0.1" length="0.04"/></geometry></visual>
    <collision><geometry><cylinder radius="0.1" length="0.04"/></geometry></collision>
  </link>
  <joint name="j_left" type="continuous">
    <parent link="base_link"/><child link="wheel_left"/>
    <origin xyz="-0.12 0.22 0" rpy="1.5708 0 0"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="j_right" type="continuous">
    <parent link="base_link"/><child link="wheel_right"/>
    <origin xyz="-0.12 -0.22 0" rpy="1.5708 0 0"/><axis xyz="0 0 1"/>
  </joint>
</robot>
''')

def show(l=""): print("URDF> "+l, flush=True)
show("importing a URDF with a DELIBERATELY missing inertial on wheel_right")
clear_stage()
# The public API is URDFImporter / URDFImporterConfig. There is no _urdf
# module in 6.0.1: importing one raises ImportError, which is what the first
# attempt did.
try:
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    cfg = URDFImporterConfig(
        urdf_path=URDF,
        usd_path="/workspace/out/urdf_import",
        merge_fixed_joints=False,
        collision_type="Convex Hull",
        fix_base=False,
    )
    imp = URDFImporter(cfg)
    path = imp.import_urdf()
    show(f"imported to prim: {path}")
except Exception as e:
    show(f"IMPORT FAILED: {type(e).__name__}: {str(e)[:120]}")
    path=None

if path:
    # import_urdf() returns the path of the USD FILE it wrote, not a prim in
    # the open stage. Traversing the current stage found 0 prims because the
    # result was never in it. Open the file it produced.
    show(f"   (that is a file on disk, not a prim: opening it)")
    st = Usd.Stage.Open(path)
    path = str(st.GetDefaultPrim().GetPath()) if st.GetDefaultPrim() else "/"
    show("")
    show("what came out, read back from the stage:")
    show(f"   {'prim':<34} {'type':<12} schemas")
    show("   "+"-"*74)
    n=0
    for p in st.Traverse():
        ps = p.GetPath().pathString
        if not ps.startswith(path): continue
        sch = [s for s in p.GetAppliedSchemas() if 'Physics' in s or 'Articulation' in s]
        if sch or p.GetTypeName() in ("Mesh","Cube","Cylinder","Xform"):
            if n < 14:
                show(f"   {ps[-34:]:<34} {str(p.GetTypeName()):<12} {sch}")
            n+=1
    show(f"   ... {n} prims in total")
    show("")
    show("the joints it created:")
    for p in st.Traverse():
        t=str(p.GetTypeName())
        if "Joint" in t:
            show(f"   {p.GetName():<20} {t}")
    show("")
    show("mass on each link (the missing inertial is the interesting one):")
    for name in ("base_link","wheel_left","wheel_right"):
        for p in st.Traverse():
            if p.GetName()==name:
                m=p.GetAttribute("physics:mass")
                show(f"   {name:<14} physics:mass = {m.Get() if m else 'NOT AUTHORED'}")
                break
print("URDF_DONE", flush=True)
sim.close()
