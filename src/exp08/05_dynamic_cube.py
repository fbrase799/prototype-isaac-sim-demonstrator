"""EXP-08 snippet 5 — new cube with rigid body + collision. New tab, then Run."""

from isaacsim.core.experimental.materials import PreviewSurfaceMaterial
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim

red_material = PreviewSurfaceMaterial("/Materials/red")
red_material.set_input_values("diffuseColor", [1.0, 0.0, 0.0])

dynamic_cube = Cube(
    paths="/dynamic_cube",
    positions=[0, -1.0, 1.0],
    sizes=0.3,
    scales=[0.6, 0.5, 0.2],
)
dynamic_cube.apply_visual_materials(red_material)
RigidPrim(paths="/dynamic_cube")
GeomPrim(paths="/dynamic_cube", apply_collision_apis=True)
