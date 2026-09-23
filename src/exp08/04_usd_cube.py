"""EXP-08 snippet 4 (optional) — visual cube via raw USD. New tab, then Run."""

import omni.usd
from pxr import Gf, UsdGeom

stage = omni.usd.get_context().get_stage()

path = "/visual_cube_usd"
cube_geom = UsdGeom.Cube.Define(stage, path)
cube_prim = stage.GetPrimAtPath(path)
size = 0.5
offset = Gf.Vec3f(1.5, -0.2, 1.0)
cube_geom.CreateSizeAttr(size)
if not cube_prim.HasAttribute("xformOp:translate"):
    UsdGeom.Xformable(cube_prim).AddTranslateOp().Set(offset)
else:
    cube_prim.GetAttribute("xformOp:translate").Set(offset)
