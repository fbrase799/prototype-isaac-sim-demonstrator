# 10.5: generating a real dataset from our scene.
#
# Everything in 10.1 to 10.4 was measured on three boxes on a plain floor,
# because a toy scene makes a mechanism legible. This lecture leaves the toy
# behind and generates from the Section 7 warehouse -- 3436 prims, 781 meshes,
# 25.6 x 40.0 x 11.3 m -- because every number changes when the scene is real,
# and the ones that change most are the ones people budget with.
#
# WHAT IS MEASURED:
#   1. Load cost and scene size, so the fixed cost of a real environment is
#      visible before a single frame is written.
#   2. Frames per second at a REAL resolution (640x480), against the 256x256
#      toy numbers from 10.1-10.4, so the scaling is honest.
#   3. Disk per frame with a working annotator set, and the extrapolation to a
#      dataset size someone would actually train on.
#   4. THE THING THE TOY SCENE COULD NOT SHOW: how many frames contain no
#      labelled object at all. In a 40-metre warehouse a randomised camera
#      spends much of its time looking at empty floor, and a dataset that is
#      mostly empty frames trains a detector to predict nothing.
#
# TRAPS CARRIED IN FROM EARLIER LECTURES, all load-bearing here:
#   - open_stage REPLACES the stage (7.1), so the environment loads FIRST and
#     everything else is created after it.
#   - rep.orchestrator.run_until_complete() never returns headless (10.1 probe);
#     step() explicitly, always.
#   - a rep.trigger.on_frame() PERSISTS on the graph (10.2), so anything
#     measured without randomisation must be measured BEFORE one exists.
from isaacsim import SimulationApp

sim = SimulationApp({"headless": True})

import glob
import os
import shutil
import time

import numpy as np
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.utils.stage import open_stage
from isaacsim.storage.native import get_assets_root_path
from pxr import Usd, UsdGeom

OUT = "/workspace/out/s10_5"
N = 24
RES = (640, 480)


def show(l=""):
    print("R105> " + l, flush=True)


show("=" * 70)
show("10.5  A REAL DATASET FROM A REAL SCENE")
show("=" * 70)
show()

# ---------------------------------------------------------------------------
# 1. THE SCENE, AND WHAT IT COSTS BEFORE ANY FRAME EXISTS
# ---------------------------------------------------------------------------
root = get_assets_root_path()
ENV = root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
show("-" * 70)
show("1. LOADING THE WAREHOUSE")
show("-" * 70)
t0 = time.perf_counter()
ok = open_stage(ENV)
load_s = time.perf_counter() - t0
show(f"  open_stage: {ok}   {load_s:.2f} s")

stage = omni.usd.get_context().get_stage()
prims = meshes = 0
for p in Usd.PrimRange(stage.GetPseudoRoot()):
    prims += 1
    if p.IsA(UsdGeom.Mesh):
        meshes += 1
show(f"  {prims} prims, {meshes} meshes")
show()
show("  That is the Section 7 warehouse, and it is the point: everything so")
show("  far was three boxes on a plane. A real environment costs seconds to")
show("  load before a single frame is generated, and that cost is paid once")
show("  per process rather than per frame -- which is an argument for long")
show("  generation runs over many short ones.")
show()

# ---------------------------------------------------------------------------
# 2. GENERATE, AT A RESOLUTION SOMEONE WOULD ACTUALLY TRAIN ON
# ---------------------------------------------------------------------------
show("-" * 70)
show("2. GENERATING AT 640x480")
show("-" * 70)

# a camera that moves through the warehouse, and props to detect
cam = rep.create.camera()
rp = rep.create.render_product(cam, RES)

# stock props with a semantic class, scattered through the aisles
props = rep.create.cube(semantics=[("class", "crate")], count=6,
                        position=(0, 0, 0.5), scale=0.6)

with rep.trigger.on_frame():
    with cam:
        rep.modify.pose(
            position=rep.distribution.uniform((-8.0, -14.0, 1.2),
                                              (8.0, 14.0, 3.0)),
            look_at=rep.distribution.uniform((-6.0, -12.0, 0.4),
                                             (6.0, 12.0, 1.6)))
    with props:
        rep.modify.pose(
            position=rep.distribution.uniform((-7.0, -13.0, 0.3),
                                              (7.0, 13.0, 1.4)),
            rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360)))

shutil.rmtree(OUT, ignore_errors=True)
os.makedirs(OUT, exist_ok=True)
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=OUT, rgb=True, bounding_box_2d_tight=True)
writer.attach([rp])

# warm-up, discarded -- 10.4's lesson: the first frames pay for pipeline init
for _ in range(3):
    rep.orchestrator.step(rt_subframes=1)
shutil.rmtree(OUT, ignore_errors=True)
os.makedirs(OUT, exist_ok=True)

t0 = time.perf_counter()
for _ in range(N):
    rep.orchestrator.step(rt_subframes=1)
gen_s = time.perf_counter() - t0
writer.detach()

files = [f for f in glob.glob(os.path.join(OUT, "**", "*"), recursive=True)
         if os.path.isfile(f)]
total_b = sum(os.path.getsize(f) for f in files)
show(f"  {N} frames in {gen_s:.2f} s   ->  {gen_s / N * 1000:.0f} ms/frame")
show(f"  {len(files)} files, {total_b / 1024 / 1024:.2f} MB "
     f"({total_b / N / 1024:.1f} KB/frame)")
show()
show(f"  The toy scene ran at 105 ms/frame at 256x256. This is a real")
show(f"  environment at 640x480, which is {640 * 480 / (256 * 256):.1f}x the pixels.")
show()

# ---------------------------------------------------------------------------
# 3. EXTRAPOLATE, BECAUSE THAT IS WHAT PEOPLE ACTUALLY DO
# ---------------------------------------------------------------------------
show("-" * 70)
show("3. WHAT 10 000 FRAMES WOULD COST")
show("-" * 70)
per_f = gen_s / N
per_b = total_b / N
for n in (1000, 10000, 100000):
    show(f"  {n:7d} frames:  {n * per_f / 60:7.1f} min   "
         f"{n * per_b / 1024 / 1024 / 1024:6.2f} GB")
show()

# ---------------------------------------------------------------------------
# 4. THE THING THE TOY SCENE COULD NOT SHOW: EMPTY FRAMES
# ---------------------------------------------------------------------------
show("-" * 70)
show("4. HOW MANY FRAMES CONTAIN NOTHING?")
show("-" * 70)
npys = sorted(glob.glob(os.path.join(OUT, "**", "bounding_box_2d_tight*.npy"),
                        recursive=True))
counts = []
for f in npys:
    try:
        counts.append(len(np.load(f, allow_pickle=True)))
    except Exception:
        counts.append(-1)
good = [c for c in counts if c >= 0]
if good:
    empty = sum(1 for c in good if c == 0)
    show(f"  {len(good)} annotated frames")
    show(f"  objects per frame: min {min(good)}, max {max(good)}, "
         f"mean {np.mean(good):.2f}")
    show(f"  frames with NO object at all: {empty}  "
         f"({empty / len(good) * 100:.1f}%)")
    show()
    if empty > 0:
        show("  THAT IS THE NUMBER THE TOY SCENE COULD NOT SHOW YOU. Three")
        show("  boxes in a two-metre frame are always visible. Six crates in a")
        show("  forty-metre warehouse, with a camera pointed randomly, are")
        show("  often not -- and a frame with no object is not a bug, it is a")
        show("  negative example. A few are useful. A dataset that is mostly")
        show("  them trains a detector to predict nothing, confidently.")
        show()
        show("  Nothing warns you. The frames render, the labels are valid,")
        show("  the file count is right. You find out at training time.")
    else:
        show("  Every frame contains at least one object here. Report that")
        show("  rather than the expected story -- but do check this on YOUR")
        show("  scene, because it is scene-dependent and it is silent.")
else:
    show("  no annotation files could be read -- report that rather than")
    show("  claiming a check that did not run")
show()

show("=" * 70)
show("done")
show("=" * 70)

sim.close()
