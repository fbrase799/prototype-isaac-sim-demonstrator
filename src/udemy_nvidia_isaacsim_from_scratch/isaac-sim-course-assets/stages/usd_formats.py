# 3.9: the usda-versus-usdc measurement the lecture quotes.
#
# The lecture refuses to repeat "binary is smaller" as received wisdom, so this
# script MEASURES it at three sizes and prints the table that goes on screen.
# Both formats are written from the SAME in-memory stage, so the only variable
# is the serialisation.
#
# The interesting result, and the reason the lecture exists: at the small end
# text WINS, because .usdc carries a structural header that costs more than the
# handful of values it stores. The crossover is about mesh data, not "big file".
#
# SimulationApp must be constructed before any pxr import: /isaac-sim/python.sh
# does not put omni.usd.libs on PYTHONPATH, so `from pxr import ...` fails with
# ModuleNotFoundError until the Kit runtime that owns the USD libraries boots.
# Same rule as 3.8. See project_udemy_isaac_section3_progress.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})

import os
from pxr import Usd, UsdGeom, UsdLux, Gf

OUT = "/workspace/out"


def build_small(stage):
    """The stage from 3.8: a handful of prims, no real geometry."""
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    g = UsdGeom.Cube.Define(stage, "/World/ground")
    g.CreateSizeAttr(1.0)
    gx = UsdGeom.Xformable(g)
    gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
    gx.AddScaleOp().Set(Gf.Vec3f(20, 20, 0.1))
    crate = UsdGeom.Xform.Define(stage, "/World/crate")
    UsdGeom.Xformable(crate).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.2))
    UsdGeom.Cube.Define(stage, "/World/crate/body").CreateSizeAttr(0.4)
    UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(1100.0)
    UsdGeom.Camera.Define(stage, "/World/shot_cam")


def build_mesh(stage, n_points):
    """A real mesh: n_points of point data, which is what actually costs bytes."""
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    mesh = UsdGeom.Mesh.Define(stage, "/World/blob")
    # A strip of quads. The geometry is arbitrary; the POINT COUNT is the
    # variable under test, so the numbers stay comparable across formats.
    pts, counts, idx = [], [], []
    for i in range(n_points):
        x = (i % 100) * 0.05
        y = (i // 100) * 0.05
        pts.append(Gf.Vec3f(x, y, 0.1 * ((i * 7919) % 13)))
    for i in range(0, n_points - 101, 1):
        if (i % 100) == 99:
            continue
        counts.append(4)
        idx.extend([i, i + 1, i + 101, i + 100])
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(idx)


CASES = [
    ("fmt_test", "3.8 stage, no geometry", build_small, None),
    ("big_300", "300-point mesh", build_mesh, 300),
    ("big_30000", "30000-point mesh", build_mesh, 30000),
]

rows = []
for stem, label, fn, arg in CASES:
    sizes = {}
    for ext in ("usda", "usdc"):
        path = os.path.join(OUT, f"{stem}.{ext}")
        if os.path.exists(path):
            os.remove(path)
        stage = Usd.Stage.CreateNew(path)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        fn(stage, arg) if arg is not None else fn(stage)
        stage.GetRootLayer().Save()
        del stage
        sizes[ext] = os.path.getsize(path)
    rows.append((stem, label, sizes["usda"], sizes["usdc"]))


def human(n):
    if n < 1024:
        return f"{n} B"
    if n < 1024 * 1024:
        return f"{n/1024:.1f} KB"
    return f"{n/1024/1024:.2f} MB"


print("")
print(f"{'stage':<12} {'contents':<24} {'.usda':>10} {'.usdc':>10}  {'ratio':>8}")
print("-" * 70)
for stem, label, a, c in rows:
    # ratio > 1 means binary is smaller, which is the advice everyone repeats.
    # Print it signed so the small case reads as the counterexample it is.
    r = a / c
    verdict = f"{r:.2f}x" if r >= 1 else f"text {c/a:.2f}x smaller"
    print(f"{stem:<12} {label:<24} {human(a):>10} {human(c):>10}  {verdict:>8}")
print("")
print("USD_FORMATS_MEASURED", flush=True)

sim.close()
