# 6.4: RTX lidar -- rays, returns and scan patterns, measured.
#
# A lidar is not a depth camera. It is a fixed set of BEAMS at known angles,
# and what it can see is decided by how those beams are laid out, not by a
# resolution setting. Three things get measured here, all off the same scan:
#
#   1. THE BEAM LAYOUT IS IN A CONFIG FILE, and the returns match it exactly.
#      Example_Rotary has 128 emitters at 32 distinct elevations (-15.0 to
#      +10.0 deg) repeated at 4 azimuth offsets, and the returns land on
#      exactly those 32 rings.
#
#   2. A TARGET GETS FEWER RETURNS AS IT GETS FURTHER AWAY, and the falloff is
#      geometric, not a range limit. The same 10 cm post is 200 columns wide at
#      2 m and 10 columns wide at 40 m, because angular width is 2*atan(w/2R).
#      Multiply by how many elevation rings still cross it and the return count
#      collapses from thousands to tens.
#
#   3. A LIDAR REPORTS RANGE ALONG THE RAY, which is the "distance to camera"
#      quantity from 6.3, not perpendicular Z. On a flat wall the returns climb
#      as 1/cos(azimuth) away from boresight -- the same factor as last lecture,
#      arriving because of how the sensor is built rather than which annotator
#      was asked.
#
# FOUR TRAPS, all hit while building this. Every one of them fails QUIETLY:
#
# 1. THE ANNOTATOR NAME IS VERSIONED. On Isaac 6.0 it is
#    "IsaacCreateRTXLidarScanBuffer". The 4.x-era name carried an
#    "RtxSensorCpu" prefix, and tutorials written against 4.2 still use it.
#    That one at least raises AnnotatorRegistryError.
#
# 2. THE SCAN BUFFER RETURNS FIRST HITS ONLY. Ring the sensor with pillars at
#    3, 8, 15 and 25 m and every single return comes back between 2.79 and
#    3.17 m: the near ring occludes everything behind it, and the scan looks
#    like a sensor with a 3 m range limit. It is not -- farRangeM is 200. Put
#    the targets on SEPARATE BEARINGS or measure nothing useful.
#
# 3. AN AZIMUTH SPAN IS NOT A FIELD OF VIEW. A first pass measured
#    -45..+45 deg and concluded the sensor swept a 90 deg wedge per step. It
#    does not: that was simply the angular extent of the geometry that existed.
#    With targets all round, every step returns the full -180..+180.
#
# 4. THE "median neighbour gap" OVER ALL POINTS IS NOT THE AZIMUTH STEP. Taken
#    across 32 interleaved elevation rings it reads 0.0119 deg; taken along ONE
#    ring it is 0.0143 deg. Mixing the rings makes the sensor look finer than
#    it is.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import math
import numpy as np
from pxr import UsdGeom, Gf, UsdLux
import omni.usd
import omni.kit.commands
from isaacsim.core.utils.stage import clear_stage
from isaacsim.core.api import World
W = World(stage_units_in_meters=1.0)


def show(l=""):
    print("LID> " + l, flush=True)


clear_stage()
st = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
w = UsdGeom.Xform.Define(st, "/World")
st.SetDefaultPrim(w.GetPrim())
UsdLux.DomeLight.Define(st, "/World/dome").CreateIntensityAttr(1100.0)

# THE POSTS GO ON SEPARATE BEARINGS. See trap 2: stacked along one bearing the
# near one hides the rest and the sensor looks range-limited.
# ALL POST BEARINGS ARE FORWARD (|bearing| <= 60), and the wall is behind at
# 180. An earlier layout fanned them 0..160 deg, which put the 20 m post at
# bearing 120 and the 40 m post at bearing 160 BEHIND the wall: the ray to the
# 40 m post crosses x = -6 at y = 2.18 m, inside the wall's +-7 m span, so it
# reported zero returns and the falloff table looked like a range cutoff. The
# posts and the wall now occupy disjoint half-planes and cannot interact.
POST_W = 0.10
RANGES = [2.0, 5.0, 10.0, 20.0, 40.0]
BEARINGS = [-60.0, -30.0, 0.0, 30.0, 60.0]
for i, (rr, bb) in enumerate(zip(RANGES, BEARINGS)):
    a = math.radians(bb)
    p = UsdGeom.Cube.Define(st, f"/World/post{i}")
    p.CreateSizeAttr(1.0)
    q = UsdGeom.Xformable(p)
    q.AddTranslateOp().Set(Gf.Vec3d(rr * math.cos(a), rr * math.sin(a), 1.0))
    q.AddScaleOp().Set(Gf.Vec3f(POST_W, POST_W, 3.0))
    p.CreateDisplayColorAttr([(0.80, 0.35, 0.30)])

# A flat wall BEHIND the sensor (bearing 180) so the 1/cos check has a surface
# of its own.
#
# ITS WIDTH IS LOAD-BEARING. At 40 m long it reached y = +-20 and stood between
# the sensor and any post on a rearward bearing, which reported ZERO returns
# and made the "returns fall off with range" table look like a range limit
# instead of geometry. With the posts moved to forward bearings the wall can no
# longer occlude them, and 14 m of it still spans +-49 deg about boresight,
# which is plenty for the 1/cos check out to 30 deg off axis.
WALLX = -6.0
c = UsdGeom.Cube.Define(st, "/World/wall")
c.CreateSizeAttr(1.0)
cx = UsdGeom.Xformable(c)
cx.AddTranslateOp().Set(Gf.Vec3d(WALLX, 0, 1.0))
cx.AddScaleOp().Set(Gf.Vec3f(0.05, 14.0, 6.0))
c.CreateDisplayColorAttr([(0.62, 0.64, 0.66)])

W.reset()
for _ in range(6):
    W.step(render=False)

CONFIG = "Example_Rotary"
ok, lidar = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/World/lidar",
    parent=None,
    config=CONFIG,
    translation=Gf.Vec3d(0.0, 0.0, 1.0),
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
)
if not ok or lidar is None:
    show("IsaacSensorCreateRtxLidar FAILED -- no table rather than a fake one")
    print("LID_DONE", flush=True)
    sim.close()
    raise SystemExit(1)

import omni.replicator.core as rep
hydra = rep.create.render_product(lidar.GetPath(), [1, 1], name="lidar_rp")
annot = rep.AnnotatorRegistry.get_annotator("IsaacCreateRTXLidarScanBuffer")
# NO outputObjectId. Asking for it without also setting the OmniLidar prim's
# 'omni:sensor:Core:auxOutputType' to BASIC/EXTRA/FULL logs an error and
# returns an empty buffer for that field; nothing here needs object ids.
annot.initialize()
annot.attach([hydra])

# TAKE THE FIRST *SETTLED* SCAN, NOT THE FIRST NON-EMPTY ONE.
#
# The buffer starts returning partial sweeps well before it returns a whole
# one: a run that broke on the first non-empty frame took 398 returns spread
# over 22 elevation rings, against 134130 over all 32 rings once settled. Both
# are "populated", and a table built on the partial one is quietly wrong -- the
# ring count alone was off by ten. So: keep stepping until the return count
# stops growing, then use that.
pts = None
last = -1
stable = 0
for i in range(400):
    W.step(render=True)
    arr = np.asarray(annot.get_data().get("data", []))
    n = arr.shape[0] if arr.ndim == 2 else 0
    if n > 0 and n == last:
        stable += 1
        if stable >= 3:
            pts = arr
            show(f"scan settled at {n} returns after {i + 1} render steps")
            break
    else:
        stable = 0
    last = n
if pts is None:
    show("SCAN BUFFER NEVER POPULATED -- no table, rather than a table of zeros")
    show("check: docker exec -u 0 isaac-sim chown -R isaac-sim:isaac-sim /isaac-sim/kit/cache")
    print("LID_DONE", flush=True)
    sim.close()
    raise SystemExit(1)

rng = np.linalg.norm(pts[:, :3], axis=1)
az = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
el = np.degrees(np.arcsin(np.clip(pts[:, 2] / np.maximum(rng, 1e-9), -1, 1)))

show(f"config '{CONFIG}': {pts.shape[0]} returns in one scan")
show("")

# ---- 1. the beam layout, recovered from the returns --------------------
rings = np.unique(np.round(el, 2))
mid = rings[len(rings) // 2]
one = np.abs(el - mid) < 0.02
ELSTEP = (rings.max() - rings.min()) / max(1, len(rings) - 1)

# Example_Rotary declares 32 elevations in its emitterStates. Anything less
# means this is still a partial sweep, and every count below would be an
# undercount presented as a measurement. Refuse rather than mislead.
EXPECT_RINGS = 32
if len(rings) != EXPECT_RINGS:
    show(f"PARTIAL SCAN: {len(rings)} elevation rings, expected {EXPECT_RINGS}")
    show("no table, rather than a table built on half a sweep")
    print("LID_DONE", flush=True)
    sim.close()
    raise SystemExit(1)

show("the beam layout is a CONFIG FILE, and the returns land on it:")
show(f"   elevation rings           : {len(rings)}")
show(f"   lowest / highest ring     : {rings.min():.2f} deg / {rings.max():.2f} deg")
show(f"   ring spacing              : {ELSTEP:.3f} deg")
show(f"   points on one ring        : {int(one.sum())}")
# THE ELEVATION LAYOUT IS QUOTED; THE AZIMUTH STEP IS NOT.
#
# The 32 rings and their -15..+10 span are stable: they come straight from
# elevationDeg in Example_Rotary.json and reproduce on every run. The azimuth
# step does not behave that way. Measured as a median neighbour gap along one
# ring it read 0.0142, 0.0131 and 0.0125 deg on three runs of this same script
# as the scene changed, and the figure derived from the config's own
# scanRateBaseHz / patternFiringRateHz (360*10/36000 = 0.1000 deg) is eight
# times larger again. Something about how firings map to returns is not what
# a naive reading of those two fields implies, and the honest thing is to not
# teach a number I cannot derive twice the same way. What IS solid, and is
# what the lecture uses, is the ANGULAR WIDTH arithmetic below: it depends
# only on the target and the range, and it predicts the measured widths.
show("")

# ---- 2. the same post, further away ------------------------------------
show(f"the SAME {POST_W * 100:.0f} cm post at five ranges, each on its own bearing:")
show(f"   {'range':>7} {'width (pred)':>13} {'width (meas)':>13} "
     f"{'rings':>7} {'returns':>9}")
show("   " + "-" * 56)
counts = []
for rr, bb in zip(RANGES, BEARINGS):
    sel = (np.abs(rng - rr) < 0.6) & (np.abs(((az - bb + 180) % 360) - 180) < 8.0)
    n = int(sel.sum())
    wid = math.degrees(2 * math.atan((POST_W / 2) / rr))
    meas = float(az[sel].max() - az[sel].min()) if n > 1 else 0.0
    half = math.degrees(math.atan(1.5 / rr))
    span = min(half, rings.max()) + min(half, -rings.min())
    nr = max(1.0, span / ELSTEP)
    show(f"   {rr:6.1f}m {wid:12.3f}d {meas:12.2f}d {nr:7.1f} {n:9d}")
    counts.append(n)
show("")
show("   The post subtends 2*atan(w/2R), which HALVES every time the range")
show("   doubles, and the elevation rings crossing it thin out the same way.")
show("   Two independent halvings is why the return count falls by roughly a")
# Built from THIS run's numbers. Hardcoding the sequence meant the prose drifted
# from the table by about 1% every run (returns jitter run to run: 4768 vs 4769,
# 1879 vs 1865). A narration that disagrees with the table on screen is the
# thing to avoid, so the sentence is generated.
show("   factor of four per doubling: " + " -> ".join(str(c) for c in counts) + ".")
show("   NOTHING here is a range limit: farRangeM is 200 m in this config, and")
show("   the 40 m post is still returning. It is pure geometry.")
show("")

# ---- 3. range along the ray, not perpendicular Z ------------------------
show("on the flat wall: a lidar reports RANGE along the ray, not perpendicular Z")
show(f"   {'azimuth':>9} {'range':>9} {'R*cos(off)':>12} {'1/cos':>8}")
show("   " + "-" * 44)
back = np.abs(np.abs(az) - 180.0) < 40.0
for target in (180.0, 170.0, 160.0, 150.0):
    off = 180.0 - target
    cand = np.where(back & (np.abs(np.abs(az) - target) < 0.3)
                    & (np.abs(el) < 0.5))[0]
    if cand.size == 0:
        continue
    k = cand[int(np.argmin(rng[cand]))]
    a, r = float(abs(az[k])), float(rng[k])
    show(f"   {a:8.2f}d {r:8.3f}m {r * math.cos(math.radians(180 - a)):11.3f}m "
         f"{1 / math.cos(math.radians(180 - a)):8.4f}")
show("")
show("   R*cos(angle off boresight) is constant and equals the wall distance:")
show("   these are along-ray ranges, the 'to camera' column from 6.3")

print("LID_DONE", flush=True)
sim.close()
