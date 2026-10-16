# 7.1: building an environment the robot can get lost in.
#
# Sections 1 to 6 built a robot and its sensors on an infinite grey plane. That
# plane is the reason none of it was interesting: a camera pointed at nothing
# returns nothing, a lidar on an empty floor returns nothing, and a navigation
# stack on open ground has no decisions to make. This section builds the world.
#
# The lecture's argument is that an environment is not scenery -- it is the
# thing that gives every sensor in the previous section something to say. Four
# things get measured, all on NVIDIA's own stock warehouse so that anyone can
# reproduce them:
#
#   1. WHAT A STOCK ENVIRONMENT ACTUALLY CONTAINS. Prim count, mesh count and
#      the world extent in metres, so "a warehouse" becomes a number you can
#      budget against rather than a vibe.
#
#   2. AN EMPTY PLANE GIVES A SENSOR NOTHING. The same lidar, in the same
#      pose, returns almost nothing on the grey plane and a full scan in the
#      warehouse. This is measured rather than asserted because it is the
#      whole reason the section exists.
#
#   3. THE ROBOT NEEDS SOMEWHERE IT CAN AND CANNOT GO. Free space is measured
#      by casting a ring of rays and reporting how far each one travels before
#      hitting something -- which is the same thing a navigation stack does.
#
#   4. LOADING IS NOT FREE. The asset streams from NVIDIA's CDN, and the time
#      it takes is a real number the student will wait for.
#
# TRAPS:
#
# - THE ASSET ROOT IS VERSIONED. Isaac 6.0 serves assets under .../Assets/
#   Isaac/6.0/, and a 4.x-era URL 404s or silently loads an older scene. Use
#   get_assets_root_path() rather than a hardcoded path.
# - open_stage REPLACES the stage, so anything created before it is gone. The
#   robot and sensors have to be added AFTER the environment loads, which is
#   the opposite order from every previous script in this course.
# - This script refuses to print a table it did not measure.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import math
import time
import numpy as np
from pxr import UsdGeom, Gf, UsdPhysics, UsdLux
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage
from isaacsim.storage.native import get_assets_root_path


def show(l=""):
    print("ENV> " + l, flush=True)


root = get_assets_root_path()
if root is None:
    show("NO ASSET ROOT -- the Isaac asset CDN is unreachable from here.")
    show("Nothing to measure; no table rather than a made-up one.")
    print("ENV_DONE", flush=True)
    sim.close()
    raise SystemExit(1)
show(f"asset root: {root}")

ENV_USD = root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"

t0 = time.time()
ok = open_stage(ENV_USD)
load_s = time.time() - t0
if not ok:
    show(f"FAILED to open {ENV_USD}")
    print("ENV_DONE", flush=True)
    sim.close()
    raise SystemExit(1)

W = World(stage_units_in_meters=1.0)
W.reset()
for _ in range(10):
    W.step(render=False)

st = omni.usd.get_context().get_stage()

# ---- 1. what is actually in there --------------------------------------
prims = list(st.Traverse())
meshes = [p for p in prims if p.IsA(UsdGeom.Mesh)]
xforms = [p for p in prims if p.IsA(UsdGeom.Xform)]
lights = [p for p in prims if p.HasAPI(UsdLux.LightAPI)]
colliders = [p for p in prims if p.HasAPI(UsdPhysics.CollisionAPI)]

# world extent from the mesh bounds
bb = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
rng = bb.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
mn, mx = rng.GetMin(), rng.GetMax()

show("")
show(f"stock environment: Simple_Warehouse/warehouse.usd")
show(f"   loaded in {load_s:.1f} s over the network")
show("")
show(f"   {'prims in the stage':>26} : {len(prims)}")
show(f"   {'meshes':>26} : {len(meshes)}")
show(f"   {'xforms':>26} : {len(xforms)}")
show(f"   {'lights':>26} : {len(lights)}")
show(f"   {'prims with a collider':>26} : {len(colliders)}")
show(f"   {'world extent, metres':>26} : "
     f"{mx[0] - mn[0]:.1f} x {mx[1] - mn[1]:.1f} x {mx[2] - mn[2]:.1f}")
show("")
show("   that is what 'a warehouse' costs: a budget you can plan against,")
show("   not a vibe. Section 7.6 comes back to what it costs in frames.")
show("")

# ---- 2. an empty plane gives a sensor nothing ---------------------------
# The same ray cast, from the same pose, in this scene and in nothing. The
# comparison is the entire argument for building an environment at all.
from isaacsim.core.utils.prims import is_prim_path_valid
import omni.physx

pxq = omni.physx.get_physx_scene_query_interface()
ORIGIN = Gf.Vec3d(0.0, 0.0, 1.0)
N_RAYS = 72
MAXD = 60.0


def ring_scan():
    """Cast N_RAYS horizontally and return the hit distances."""
    out = []
    for i in range(N_RAYS):
        a = 2.0 * math.pi * i / N_RAYS
        d = carb_raycast(ORIGIN, Gf.Vec3d(math.cos(a), math.sin(a), 0.0))
        out.append(d)
    return np.asarray(out, dtype=float)


def carb_raycast(origin, direction):
    hit = pxq.raycast_closest(
        [origin[0], origin[1], origin[2]],
        [direction[0], direction[1], direction[2]], MAXD)
    if hit and hit.get("hit", False):
        return float(hit["distance"])
    return float("inf")


d = ring_scan()
hits = np.isfinite(d)
if not hits.any():
    show("NO RAY HIT ANYTHING -- the colliders did not load; no table.")
    print("ENV_DONE", flush=True)
    sim.close()
    raise SystemExit(1)

show(f"a ring of {N_RAYS} rays cast from (0, 0, 1.0), out to {MAXD:.0f} m:")
show(f"   {'rays that hit something':>26} : {int(hits.sum())} of {N_RAYS}")
show(f"   {'nearest wall':>26} : {d[hits].min():.2f} m")
show(f"   {'furthest hit':>26} : {d[hits].max():.2f} m")
show(f"   {'median free distance':>26} : {np.median(d[hits]):.2f} m")
show("")

# THE EMPTY-PLANE COMPARISON IS MEASURED, NOT ASSERTED. A first version simply
# claimed "0 of 72 on the grey plane" in prose. It is the lecture's whole
# argument, so it gets its own scan on a stage that really is a bare ground
# plane: same origin, same ring, same 60 m limit.
from isaacsim.core.utils.stage import create_new_stage
create_new_stage()
st2 = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(st2, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st2, 1.0)
w2 = UsdGeom.Xform.Define(st2, "/World")
st2.SetDefaultPrim(w2.GetPrim())
gp = UsdGeom.Cube.Define(st2, "/World/ground")
gp.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(gp)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
gx.AddScaleOp().Set(Gf.Vec3f(400.0, 400.0, 1.0))
UsdPhysics.CollisionAPI.Apply(gp.GetPrim())
W2 = World(stage_units_in_meters=1.0)
W2.reset()
for _ in range(10):
    W2.step(render=False)
d0 = ring_scan()
h0 = np.isfinite(d0)
show("the SAME ring, on the infinite grey plane from Sections 1 to 6:")
show(f"   {'rays that hit something':>26} : {int(h0.sum())} of {N_RAYS}")
show(f"   {'warehouse, for comparison':>26} : {int(hits.sum())} of {N_RAYS}")
show("")
show("   That is why the lidar lecture had to build its own posts, and why a")
show("   navigation stack has nothing to plan around on an empty floor.")
show("")

# ---- 3. free space, which is what a planner actually sees ---------------
show("the same ring, read as free space in each direction:")
show(f"   {'bearing':>10} {'free distance':>15}")
show("   " + "-" * 30)
for deg in (0, 45, 90, 135, 180, 225, 270, 315):
    i = int(round(deg / 360.0 * N_RAYS)) % N_RAYS
    v = d[i]
    show(f"   {deg:9d}d {('%.2f m' % v) if math.isfinite(v) else 'open':>15}")
show("")
show("   an environment worth simulating is one where those numbers DIFFER.")
show("   Equal in every direction is an empty room, and an empty room is a")
show("   robot with no decisions to make.")

print("ENV_DONE", flush=True)
sim.close()
