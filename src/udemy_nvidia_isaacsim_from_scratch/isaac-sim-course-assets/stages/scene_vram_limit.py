# 7.7: FAILURE -- the scene that will not fit in VRAM.
#
# This is the failure lecture for Section 7, and it is deliberately the
# CAUTIOUS version of that experiment. The honest thing to teach is what
# running out of video memory looks like from the inside, and the honest thing
# to NOT do is hard-lock the machine recording the course to get the footage.
#
# So this script WALKS TOWARD the edge and stops, rather than driving off it:
#
#   1. It loads UNIQUE geometry, not references. 7.6 measured that references
#      cost almost no memory (500 props, +83 MiB), which is exactly why they
#      are the wrong tool for running out. Unique meshes are what actually
#      consume VRAM.
#
#   2. It samples nvidia-smi after every batch and reports the curve.
#
#   3. IT STOPS AT A CEILING IT SETS ITSELF, well below the card's limit, and
#      reports how much headroom was left and what the trend says would have
#      happened. An extrapolation you can check beats a crash you cannot.
#
# WHY NOT JUST RUN OUT: a laptop dGPU sharing a card with the desktop
# compositor does not fail politely. The documented outcome on this machine is
# the whole session going down, which costs far more than the footage is worth.
# The lecture says this out loud rather than pretending the ceiling is a
# technical limitation.
#
# TRAPS:
#
# - nvidia-smi reports DEVICE memory, so the browser and the desktop are in
#   the number too. The baseline is measured first and everything is reported
#   as a delta over it.
# - Unique geometry has to actually BE unique. Referencing the same USD, or
#   even instancing the same points, shares memory and nothing grows.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import math
import subprocess
import numpy as np
from pxr import UsdGeom, Gf, UsdLux, Vt
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import create_new_stage

CEILING_MIB = 5200          # hard stop; the card has 6141 and the desktop needs some
BATCH = 4
MAX_BATCHES = 60
# MESH SIZE IS THE WHOLE EXPERIMENT, and the first attempt got it wrong.
# 480 spheres at 64x64 (4225 verts, 0.05 MB of points each) moved VRAM by
# +66 MiB total -- about 0.14 MiB per mesh, most of it noise, and the "slope"
# fitted to that was meaningless. Filling 3.2 GB at that size needs ~23000
# meshes. At 384x384 each mesh is ~148k verts and about 1.8 MB of raw points,
# so a few hundred of them actually move the number.
RINGS = SEGS = 384


def show(l=""):
    print("VRM> " + l, flush=True)


def vram():
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.used,memory.total",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=10).stdout.strip()
        u, t = [int(x) for x in out.split(",")]
        return u, t
    except Exception:
        return None, None


create_new_stage()
st = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
w = UsdGeom.Xform.Define(st, "/World")
st.SetDefaultPrim(w.GetPrim())
UsdLux.DomeLight.Define(st, "/World/dome").CreateIntensityAttr(1000.0)

RES = (640, 360)
cam = UsdGeom.Camera.Define(st, "/World/cam")
cam.CreateFocalLengthAttr(18.0)
cam.CreateHorizontalApertureAttr(20.955)
cam.CreateVerticalApertureAttr(20.955 * RES[1] / RES[0])
UsdGeom.Xformable(cam).AddTransformOp().Set(
    Gf.Matrix4d().SetLookAt(Gf.Vec3d(-30, -30, 18), Gf.Vec3d(0, 0, 0),
                            Gf.Vec3d(0, 0, 1)).GetInverse())

W = World(stage_units_in_meters=1.0)
W.reset()
from isaacsim.sensors.camera import Camera
cm = Camera(prim_path="/World/cam", resolution=RES)
cm.initialize()
ok = False
for _ in range(120):
    W.step(render=True)
    a = np.asarray(cm.get_rgb())
    if a.ndim == 3 and a.size:
        ok = True
        break
if not ok:
    show("CAMERA NEVER RETURNED A FRAME -- no table rather than zeros.")
    print("VRM_DONE", flush=True)
    sim.close()
    raise SystemExit(1)

base_used, total = vram()
show(f"card {total} MiB, {base_used} MiB already in use.")
show(f"hard ceiling for this run: {CEILING_MIB} MiB. This script STOPS there.")
show("")
show("loading UNIQUE geometry -- not references. 7.6 measured that 500")
show("referenced props cost +83 MiB, which is precisely why references are")
show("the wrong tool for filling a card. Each mesh below is its own vertices.")
show("")
show(f"   {'meshes':>8} {'M triangles':>12} {'VRAM MiB':>10} {'over base':>10} "
     f"{'headroom':>10}")
show("   " + "-" * 56)

made = 0
rows = []
stopped = "completed all batches"


def unique_sphere(path, seed, rings=RINGS, segs=SEGS):
    """A distinct mesh: points are perturbed per-seed so nothing is shared."""
    pts, idx, cnt = [], [], []
    rng = np.random.default_rng(seed)
    jig = rng.uniform(0.98, 1.02, size=(rings + 1, segs + 1))
    for i in range(rings + 1):
        th = math.pi * i / rings
        for j in range(segs + 1):
            ph = 2 * math.pi * j / segs
            r = jig[i, j]
            pts.append(Gf.Vec3f(r * math.sin(th) * math.cos(ph),
                                r * math.sin(th) * math.sin(ph),
                                r * math.cos(th)))
    for i in range(rings):
        for j in range(segs):
            a = i * (segs + 1) + j
            b = a + segs + 1
            idx += [a, b, b + 1, a + 1]
            cnt.append(4)
    m = UsdGeom.Mesh.Define(st, path)
    m.CreatePointsAttr(Vt.Vec3fArray(pts))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    m.CreateFaceVertexCountsAttr(Vt.IntArray(cnt))
    x = UsdGeom.Xformable(m)
    x.AddTranslateOp().Set(Gf.Vec3d((seed % 20) * 2.4 - 24,
                                    (seed // 20) * 2.4 - 24, 1.0))
    return rings * segs * 2


tris_each = 0
for b in range(MAX_BATCHES):
    for k in range(BATCH):
        tris_each = unique_sphere(f"/World/m_{made:04d}", made)
        made += 1
    for _ in range(6):
        W.step(render=True)
    used, _ = vram()
    if used is None:
        show("   nvidia-smi unavailable mid-run; stopping rather than guessing")
        stopped = "nvidia-smi unavailable"
        break
    rows.append((made, made * tris_each / 1e6, used, used - base_used))
    show(f"   {made:8d} {made * tris_each / 1e6:12.2f} {used:10d} "
         f"{used - base_used:+10d} {total - used:10d}")
    if used >= CEILING_MIB:
        stopped = f"hit the self-imposed {CEILING_MIB} MiB ceiling"
        break

show("")
show(f"stopped: {stopped}")
if len(rows) >= 3:
    xs = np.array([r[0] for r in rows], dtype=float)
    ys = np.array([r[2] for r in rows], dtype=float)
    slope = float(np.polyfit(xs, ys, 1)[0])
    remaining = total - rows[-1][2]
    show(f"   each unique mesh cost about {slope:.2f} MiB of VRAM.")
    show(f"   {remaining} MiB of card was left when this run stopped, so about")
    show(f"   {int(remaining / max(slope, 1e-6))} more meshes would have "
         f"exhausted it.")
show("")
show("   THAT LAST NUMBER IS AN EXTRAPOLATION AND IT IS DELIBERATE. Running a")
show("   laptop dGPU that also drives the desktop completely out of memory")
show("   does not fail politely: it can take the whole session down. The")
show("   lecture shows you the slope and the arithmetic instead, which is the")
show("   part you can actually use when budgeting your own scene.")

print("VRM_DONE", flush=True)
sim.close()
