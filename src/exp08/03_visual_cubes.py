"""EXP-08 snippet 3 — new Script Editor tab, then Run.

Yellow cube is visual-only. Cyan cube gets physics in snippet 5.
"""

from isaacsim.core.experimental.materials import PreviewSurfaceMaterial
from isaacsim.core.experimental.objects import Cube

yellow_material = PreviewSurfaceMaterial("/Materials/yellow")
yellow_material.set_input_values("diffuseColor", [1.0, 1.0, 0.0])

cyan_material = PreviewSurfaceMaterial("/Materials/cyan")
cyan_material.set_input_values("diffuseColor", [0.0, 1.0, 1.0])

visual_cube = Cube(
    paths="/visual_cube",
    positions=[0, 0.5, 0.5],
    sizes=0.3,
)
visual_cube.apply_visual_materials(yellow_material)

test_cube = Cube(
    paths="/test_cube",
    positions=[0, -0.5, 0.5],
    sizes=0.3,
)
test_cube.apply_visual_materials(cyan_material)
