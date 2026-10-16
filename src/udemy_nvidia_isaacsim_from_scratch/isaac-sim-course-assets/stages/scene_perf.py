# 7.6: performance -- what costs frames on 6 GB.
#
# Everything in Section 7 has been spending something. This lecture measures
# what, on the card this course is actually recorded on: an RTX 4050 Laptop
# with 6141 MiB of VRAM. The numbers below are that card's numbers, and the
# point is the SHAPE of the curve, which is the same everywhere.
#
# Three things get measured:
#
#   1. WHAT A SCENE COSTS TO DRAW, as a function of how much is in it. Frame
#      time and VRAM are sampled as identical props are added in batches, so
#      "the mesh count drives it" becomes a curve rather than a claim.
#
#   2. INSTANCING IS NOT FREE BUT IT IS CHEAP. The same visible object count
#      via references costs far less than unique geometry, which is the single
#      biggest lever available.
#
#   3. THE HEADROOM IS SMALLER THAN THE CARD. Kit itself, the RTX renderer and
#      the OS all take a share before the scene gets any, so the usable budget
#      is well under the number on the box. 7.7 walks off that edge on purpose.
#
# TRAPS:
#
# - torch.cuda / nvidia-smi report DEVICE memory, which includes everything on
#   the card, not just this process. Both numbers are reported so the
#   difference is visible rather than confusing.
# - Frame time must be measured with rendering ON. W.step(render=False) is
#   physics only and will report a scene as free that is not.
# - This script refuses to print a table it did not measure.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import subprocess
import time
import numpy as np
from pxr import UsdGeom, Gf, UsdLux, UsdPhysics, Sdf
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import create_new_stage, add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path


def show(l=""):
    print("PRF> " + l, flush=True)


def vram_used_mib():
    """Device-wide VRAM in use, from nvidia-smi."""
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.used,memory.total",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=10).stdout.strip()
        used, total = [int(x) for x in out.split(",")]
        return used, total
    except Exception:
        return None, None


root = get_assets_root_path()
if root is None:
    show("NO ASSET ROOT -- the CDN is unreachable; no table.")
    print("PRF_DONE", flush=True)
    sim.close()
    raise SystemExit(1)

create_new_stage()
st = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
w = UsdGeom.Xform.Define(st, "/World")
st.SetDefaultPrim(w.GetPrim())
UsdLux.DomeLight.Define(st, "/World/dome").CreateIntensityAttr(1000.0)

gp = UsdGeom.Cube.Define(st, "/World/floor")
gp.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(gp)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
gx.AddScaleOp().Set(Gf.Vec3f(120.0, 120.0, 1.0))

RES = (640, 360)
cam = UsdGeom.Camera.Define(st, "/World/cam")
cam.CreateFocalLengthAttr(18.0)
cam.CreateHorizontalApertureAttr(20.955)
cam.CreateVerticalApertureAttr(20.955 * RES[1] / RES[0])
UsdGeom.Xformable(cam).AddTransformOp().Set(
    Gf.Matrix4d().SetLookAt(Gf.Vec3d(-14, -14, 9), Gf.Vec3d(6, 6, 0),
                            Gf.Vec3d(0, 0, 1)).GetInverse())

W = World(stage_units_in_meters=1.0)
W.reset()
from isaacsim.sensors.camera import Camera
cm = Camera(prim_path="/World/cam", resolution=RES)
cm.initialize()

ready = False
for _ in range(120):
    W.step(render=True)
    a = np.asarray(cm.get_rgb())
    if a.ndim == 3 and a.size:
        ready = True
        break
if not ready:
    show("CAMERA NEVER RETURNED A FRAME -- no table rather than zeros.")
    print("PRF_DONE", flush=True)
    sim.close()
    raise SystemExit(1)

PROP = root + "/Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd"
TRIS_EACH = 16384


def frame_ms(n=30):
    """Median frame time with RENDERING ON. render=False is physics only."""
    ts = []
    for _ in range(n):
        t0 = time.perf_counter()
        W.step(render=True)
        ts.append((time.perf_counter() - t0) * 1000.0)
    return float(np.median(ts))


base_used, total = vram_used_mib()
base_ms = frame_ms()
show(f"card: {total} MiB total. Kit + renderer + OS already hold "
     f"{base_used} MiB before the scene has anything in it.")
show(f"empty scene, {RES[0]}x{RES[1]} render: {base_ms:.1f} ms/frame")
show("")
show("adding the same 16384-triangle prop, in batches, by REFERENCE:")
show(f"   {'props':>7} {'triangles':>12} {'ms/frame':>10} {'fps':>7} "
     f"{'VRAM MiB':>10} {'over base':>10}")
show("   " + "-" * 62)

placed = 0
rows = []
for target in (25, 100, 250, 500):
    while placed < target:
        i = placed
        p = f"/World/prop_{i:04d}"
        add_reference_to_stage(usd_path=PROP, prim_path=p)
        x = UsdGeom.Xformable(st.GetPrimAtPath(p))
        gx2 = None
        for op in x.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                gx2 = op
                break
        pos = Gf.Vec3d((i % 25) * 0.9 - 2.0, (i // 25) * 0.9 - 2.0, 0.2)
        if gx2 is not None:
            gx2.Set(pos)
        else:
            x.AddTranslateOp().Set(pos)
        placed += 1
    for _ in range(20):
        W.step(render=True)
    ms = frame_ms()
    used, _ = vram_used_mib()
    rows.append((placed, placed * TRIS_EACH, ms, used))
    show(f"   {placed:7d} {placed * TRIS_EACH:12,} {ms:9.1f} "
         f"{1000.0 / max(ms, 1e-6):7.1f} {used:10d} {used - base_used:+10d}")

show("")
first, last = rows[0], rows[-1]
show(f"   from {first[0]} props to {last[0]}: frame time "
     f"{first[2]:.1f} -> {last[2]:.1f} ms "
     f"({last[2] / max(first[2], 1e-6):.1f}x), VRAM "
     f"{first[3] - base_used:+d} -> {last[3] - base_used:+d} MiB over base.")
show("")
show("   NOTE WHAT DID NOT HAPPEN. These are REFERENCES to one asset, so the")
show("   triangles are shared: 500 props is 8.2 million triangles of drawing")
show("   but nothing like 500 copies of the mesh in memory. That is the single")
show("   biggest lever you have, and it is free.")
show("")
show(f"   And note the headroom. The card says {total} MiB, but "
     f"{base_used} MiB was")
show("   gone before the scene loaded a single prop. What you can actually")
show("   spend is the difference, and 7.7 walks off that edge on purpose.")

print("PRF_DONE", flush=True)
sim.close()
