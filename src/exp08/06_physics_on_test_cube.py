"""EXP-08 snippet 6 — add physics to the existing cyan cube. New tab, then Run."""

from isaacsim.core.experimental.prims import GeomPrim, RigidPrim

RigidPrim(paths="/test_cube")
GeomPrim(paths="/test_cube", apply_collision_apis=True)
