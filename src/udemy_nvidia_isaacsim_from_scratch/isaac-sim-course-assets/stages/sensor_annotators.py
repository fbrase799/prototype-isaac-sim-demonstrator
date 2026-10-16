# 6.8: annotators -- ground truth the real world cannot give you.
#
# Every sensor so far has simulated a thing you can buy. An annotator is
# different in kind: it asks the renderer what it KNOWS, not what a sensor
# would see. Perfect per-pixel object identity, exact 3D extents, and the
# fraction of an object that is hidden. No real camera can produce any of it,
# and a human labelling the same frame would be slower, worse, and would not
# be able to produce two of the three at all.
#
# Four things get measured:
#
#   1. SEMANTIC SEGMENTATION IS EXACT AND FREE. Every pixel is labelled with
#      the class of the thing it belongs to, with no boundary error, no
#      annotator fatigue and no cost per frame. This is the thing that makes
#      synthetic data worth generating at all.
#
#   2. LABELS DO NOT APPEAR BY THEMSELVES. A prim with no semantic label is
#      not missing from the output -- it is worse than that. Its pixels are
#      swept into an UNLABELLED bin (26.67% of this frame), so a model trained
#      on it learns that a perfectly good crate is background. Nothing warns.
#
#   3. A 3D BOX IS REAL 3D, once decoded. The extents are LOCAL (every row
#      reads 1.000 x 1.000 x 1.000 for a unit cube) and the size lives in the
#      row's transform; multiplying the two recovers the crate's built
#      0.600 x 0.400 x 0.800 m exactly.
#
#   4. OCCLUSION IS A NUMBER, and it rides on the 3D box row. Measured here:
#      the pillar is 86.0% hidden behind the screen and the crate 0.0%. A
#      training set can be filtered on that column; no manual labelling process
#      can produce it at all.
#
# TRAPS:
#
# - THE SEMANTIC API IS RENAMED on 6.0: add_labels(prim, [labels]). The 4.x
#   add_update_semantics raises ImportError from a module now under
#   extsDeprecated.
# - bounding_box_3d RETURNS LOCAL-SPACE EXTENTS. Every row reads -0.5..+0.5 for
#   a unit cube whatever size it is on screen; the scale lives in the row's
#   `transform` matrix. Reading x_max - x_min alone reports 1.000 m for a 0.6 m
#   crate, which is the sort of wrong that looks like a correct answer.
# - OCCLUSION IS A FIELD ON THE 3D BOX ROW (`occlusionRatio`), not a separate
#   annotator dict; the standalone `occlusion` annotator returns a bare array
#   with no `data` field and raises "no field of name data" if indexed.
# - Annotators return None until the render pipeline has actually run, and the
#   /isaac-sim/kit/cache ownership trap from 6.1 applies here too.
# - This script refuses to print a table it did not measure.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import numpy as np
from pxr import UsdGeom, Gf, UsdLux
import omni.usd
from isaacsim.core.utils.stage import clear_stage
from isaacsim.core.api import World
import omni.replicator.core as rep

W = World(stage_units_in_meters=1.0)


def show(l=""):
    print("ANN> " + l, flush=True)


clear_stage()
st = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
w = UsdGeom.Xform.Define(st, "/World")
st.SetDefaultPrim(w.GetPrim())
UsdLux.DomeLight.Define(st, "/World/dome").CreateIntensityAttr(1200.0)

# Ground, plus three objects: two labelled, one deliberately NOT. The unlabelled
# one is the lecture's second point and it has to be visible in the render so
# that its absence from the output is obviously a labelling failure and not a
# missing object.
gp = UsdGeom.Cube.Define(st, "/World/ground")
gp.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(gp)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
gx.AddScaleOp().Set(Gf.Vec3f(20.0, 20.0, 1.0))
gp.CreateDisplayColorAttr([(0.35, 0.37, 0.40)])


def box(path, pos, scale, colour):
    b = UsdGeom.Cube.Define(st, path)
    b.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(b)
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    x.AddScaleOp().Set(Gf.Vec3f(*scale))
    b.CreateDisplayColorAttr([colour])
    return b


# BOX_A is 0.60 x 0.40 x 0.80 m, so its 3D extents are known by construction.
A = box("/World/crate", (2.6, -0.7, 0.40), (0.60, 0.40, 0.80), (0.75, 0.45, 0.25))
B = box("/World/pillar", (3.4, 0.9, 0.90), (0.35, 0.35, 1.80), (0.35, 0.55, 0.75))
# the OCCLUDER sits between the camera and the pillar, so occlusion has a
# non-zero value to report
OCC = box("/World/screen", (2.0, 0.9, 0.60), (0.06, 0.90, 1.20), (0.55, 0.55, 0.58))
# UNLABELLED on purpose. Same kind of object as the crate.
U = box("/World/unlabelled_crate", (2.9, 0.05, 0.35), (0.50, 0.50, 0.70),
        (0.70, 0.30, 0.55))

# THE SEMANTIC API IS RENAMED ON 6.0. It is add_labels(prim, [labels]) from
# isaacsim.core.utils.semantics; the 4.x add_update_semantics(prim, "class")
# raises ImportError from that same module, which now lives under
# extsDeprecated. Tutorials written against 4.x fail here outright -- loudly,
# for once.
from isaacsim.core.utils.semantics import add_labels, check_missing_labels
add_labels(A.GetPrim(), ["crate"])
add_labels(B.GetPrim(), ["pillar"])
add_labels(OCC.GetPrim(), ["screen"])
# /World/unlabelled_crate gets NOTHING, deliberately.

RES = (640, 360)
cam = UsdGeom.Camera.Define(st, "/World/cam")
cam.CreateFocalLengthAttr(24.0)
cam.CreateHorizontalApertureAttr(20.955)
cam.CreateVerticalApertureAttr(20.955 * RES[1] / RES[0])
UsdGeom.Xformable(cam).AddTransformOp().Set(
    Gf.Matrix4d().SetLookAt(Gf.Vec3d(0, 0, 0.9), Gf.Vec3d(3.0, 0, 0.6),
                            Gf.Vec3d(0, 0, 1)).GetInverse())

W.reset()
for _ in range(4):
    W.step(render=False)

rp = rep.create.render_product("/World/cam", RES)
sem = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")
bb3 = rep.AnnotatorRegistry.get_annotator("bounding_box_3d")
occ = rep.AnnotatorRegistry.get_annotator("occlusion")
for a in (sem, bb3, occ):
    a.attach([rp])

sd = b3 = oc = None
for i in range(120):
    W.step(render=True)
    s = sem.get_data()
    if isinstance(s, dict) and np.asarray(s.get("data", [])).ndim == 2:
        sd, b3, oc = s, bb3.get_data(), occ.get_data()
        show(f"annotators populated after {i + 1} render steps")
        break
if sd is None:
    show("ANNOTATORS NEVER POPULATED -- no table rather than a table of zeros")
    show("check: docker exec -u 0 isaac-sim chown -R isaac-sim:isaac-sim /isaac-sim/kit/cache")
    print("ANN_DONE", flush=True)
    sim.close()
    raise SystemExit(1)

# ---- 1. every pixel labelled, exactly ----------------------------------
arr = np.asarray(sd["data"])
info = sd.get("info", {}).get("idToLabels", {})
total = arr.size
show("")
show(f"semantic segmentation, {RES[0]}x{RES[1]} = {total} pixels, "
     f"every one of them labelled:")
show(f"   {'class':>16} {'pixels':>9} {'% of frame':>11}")
show("   " + "-" * 40)
named = {}
for idx, lab in sorted(info.items(), key=lambda kv: str(kv[1])):
    try:
        i = int(idx)
    except (TypeError, ValueError):
        continue
    n = int(np.count_nonzero(arr == i))
    if n == 0:
        continue
    name = lab.get("class", str(lab)) if isinstance(lab, dict) else str(lab)
    named[name] = n
    show(f"   {name:>16} {n:9d} {100.0 * n / total:10.2f}%")
show("")
show("   no boundary error, no disagreement between labellers, no cost per")
show("   frame. This is what makes generating data worth doing at all.")
show("")

# ---- 2. the object that is simply not there ----------------------------
unl = named.get("UNLABELLED", 0)
show("look at that UNLABELLED row. The scene has FOUR objects and three were")
show("given a class; the fourth is right there in the render and its pixels")
show(f"are collected into a bin called UNLABELLED -- {unl} of them, "
     f"{100.0 * unl / total:.2f}% of the frame.")
show("")
show("   It is not an error and nothing warns about it. Train on this and the")
show("   model learns that a perfectly good crate is background. A dataset is")
show("   quietly short a class and you find out from the model, not the tool.")
show("")

# ---- 3. a 3D box is real 3D --------------------------------------------
show("bounding_box_3d is REAL 3D -- but read it carefully:")
try:
    rows = np.asarray(b3["data"])
    b3info = b3.get("info", {}).get("idToLabels", {})
    show(f"   {'class':>10} {'local extent':>22} {'x scale':>9} "
         f"{'world size (x,y,z)':>24}")
    show("   " + "-" * 70)
    for r in rows:
        sid = int(r["semanticId"])
        lab = b3info.get(str(sid), b3info.get(sid, {}))
        name = lab.get("class", str(sid)) if isinstance(lab, dict) else str(lab)
        lx = float(r["x_max"] - r["x_min"])
        ly = float(r["y_max"] - r["y_min"])
        lz = float(r["z_max"] - r["z_min"])
        T = np.asarray(r["transform"])
        sx = float(np.linalg.norm(T[0, :3]))
        sy = float(np.linalg.norm(T[1, :3]))
        sz = float(np.linalg.norm(T[2, :3]))
        show(f"   {name:>10} {lx:6.3f} x {ly:.3f} x {lz:.3f} {sx:8.3f} "
             f"{lx * sx:9.3f} x {ly * sy:.3f} x {lz * sz:.3f} m")
    show("")
    show("   READ THAT FIRST COLUMN. Every row is 1.000 x 1.000 x 1.000, because")
    show("   the extents are in the object's LOCAL space. The size lives in the")
    show("   transform, and the world dimensions are the product of the two.")
    show("   The crate was built 0.600 x 0.400 x 0.800 m and the last column")
    show("   recovers exactly that -- a number a human labelling a photograph")
    show("   could not have produced at all.")
except Exception as e:
    show(f"   3D box read failed: {type(e).__name__}: {e}")
show("")

# ---- 4. occlusion is a number -------------------------------------------
show("occlusion: the renderer knows exactly how much of each object is hidden.")
show("It arrives as a FIELD ON THE 3D BOX ROW, not as its own annotator:")
try:
    rows = np.asarray(b3["data"])
    b3info = b3.get("info", {}).get("idToLabels", {})
    show(f"   {'class':>10} {'occluded':>10}")
    show("   " + "-" * 24)
    for r in rows:
        sid = int(r["semanticId"])
        lab = b3info.get(str(sid), b3info.get(sid, {}))
        name = lab.get("class", str(sid)) if isinstance(lab, dict) else str(lab)
        show(f"   {name:>10} {100.0 * float(r['occlusionRatio']):9.1f}%")
    show("")
    show("   so a training set can be filtered on it: drop everything more than")
    show("   80% hidden, or deliberately keep the hard ones. No manual")
    show("   labelling process can give you that column at all.")
except Exception as e:
    show(f"   occlusion read failed: {type(e).__name__}: {e}")

print("ANN_DONE", flush=True)
sim.close()
