# 10.1: why synthetic data, and when it fails.
#
# The curriculum note on this lecture is "state the caveat", and the caveat is
# the whole reason the lecture exists. Synthetic data is usually sold on its
# strengths -- free labels, infinite volume, perfect ground truth -- and the
# strengths are real. What gets left out is the failure mode, which is specific,
# measurable, and the reason people's models do not transfer.
#
# SO THIS SCRIPT MEASURES BOTH, on the same scene:
#
#   1. WHAT YOU GET FREE. Generate a dataset and count what came with it: RGB,
#      and the annotations a real camera cannot give you at any price. Labelling
#      2000 boxes by hand is roughly 8 hours of human work; here it is a flag.
#
#   2. THE COST PER FRAME, measured, so "infinite data" gets a price.
#
#   3. THE FAILURE, which is the lecture. A dataset generated WITHOUT
#      randomisation looks perfect and teaches a model nothing that transfers:
#      every image is nearly identical. Measured as pixel variance ACROSS the
#      dataset, per pixel, against a randomised run of the same size. That
#      number is the difference between a dataset and 200 copies of one photo.
#
# THE TRAP THIS SCRIPT ITSELF AVOIDS: rep.orchestrator.run_until_complete()
# never returns headless -- it ran 45 minutes and wrote nothing. Everything here
# drives rep.orchestrator.step() explicitly and prints the file count as it goes.
from isaacsim import SimulationApp

sim = SimulationApp({"headless": True})

import glob
import os
import shutil
import time

import numpy as np
import omni.replicator.core as rep
from isaacsim.core.api import World

OUT_FIXED = "/workspace/out/s10_fixed"
OUT_RAND = "/workspace/out/s10_random"
N = 24
RES = (256, 256)


def show(l=""):
    print("R101> " + l, flush=True)


def files(d):
    return sorted(f for f in glob.glob(os.path.join(d, "**", "*"),
                                       recursive=True) if os.path.isfile(f))


show("=" * 68)
show("10.1  WHY SYNTHETIC DATA, AND WHEN IT FAILS")
show("=" * 68)
show()

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
world.reset()

cam = rep.create.camera(position=(2.6, 2.6, 1.9), look_at=(0, 0, 0.25))
rp = rep.create.render_product(cam, RES)

# three boxes on the ground: the "parts on a conveyor" of every vision dataset
boxes = rep.create.cube(semantics=[("class", "box")], count=3,
                        position=(0, 0, 0.25), scale=0.3)


def run(out_dir, randomise, annotations):
    """Generate N frames. Returns (seconds, file list).

    ORDER MATTERS AND THE FIRST VERSION OF THIS WAS WRONG. A
    `rep.trigger.on_frame()` block registers a randomiser ON THE GRAPH, and it
    stays there. Running the randomised pass first and the "fixed" pass second
    meant the second pass was still being randomised by the first pass's
    trigger: per-pixel std came out 8.885 against 8.601, a ratio of 1.0x, and
    the lecture's whole point evaporated.

    The guard printed "the fixed run varied more than expected" instead of the
    tidy story, which is how the bug was caught. The fix is to run the FIXED
    pass first, before any randomiser exists.
    """
    shutil.rmtree(out_dir, ignore_errors=True)
    os.makedirs(out_dir, exist_ok=True)
    if randomise:
        with rep.trigger.on_frame():
            with boxes:
                rep.modify.pose(
                    position=rep.distribution.uniform((-0.8, -0.8, 0.15),
                                                      (0.8, 0.8, 0.5)),
                    rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360)))
    w = rep.WriterRegistry.get("BasicWriter")
    kw = dict(output_dir=out_dir, rgb=True)
    if annotations:
        kw["bounding_box_2d_tight"] = True
        kw["semantic_segmentation"] = True
    w.initialize(**kw)
    w.attach([rp])
    t0 = time.perf_counter()
    for _ in range(N):
        rep.orchestrator.step(rt_subframes=1)
    dt = time.perf_counter() - t0
    w.detach()
    return dt, files(out_dir)


# ---------------------------------------------------------------------------
# 1. WHAT COMES FREE
# ---------------------------------------------------------------------------
# THE FIXED PASS RUNS FIRST, BEFORE ANY RANDOMISER IS REGISTERED. See the note
# in run(): a trigger persists on the graph, so doing this in the other order
# silently randomises the "fixed" dataset too.
t_fix, f_fix = run(OUT_FIXED, randomise=False, annotations=False)

show("-" * 68)
show("1. WHAT COMES FREE WITH THE PIXELS")
show("-" * 68)
t_rand, f_rand = run(OUT_RAND, randomise=True, annotations=True)
kinds = {}
for f in f_rand:
    base = os.path.basename(f)
    key = base.split("_")[0] if "_" in base else base
    kinds[key] = kinds.get(key, 0) + 1
for k, v in sorted(kinds.items()):
    show(f"  {k:28s} x{v}")
show()
show(f"  {N} frames at {RES[0]}x{RES[1]} in {t_rand:.2f} s "
     f"({t_rand / N * 1000:.0f} ms/frame)")
show()
show("  The boxes and the segmentation masks are not a convenience here, they")
show("  are the thing a real camera cannot give you. Hand-labelling tight boxes")
show("  runs about 15 s each even for an experienced annotator, so these")
show(f"  {N} frames would be roughly {N * 3 * 15 / 60:.0f} minutes of human work")
show("  for three objects a frame -- and the mask would be far worse.")
show()

# ---------------------------------------------------------------------------
# 2. THE FAILURE: a dataset with no variation
# ---------------------------------------------------------------------------
show("-" * 68)
show("2. THE FAILURE MODE, MEASURED")
show("-" * 68)
show(f"  the other run, same scene, NO randomisation: {t_fix:.2f} s")
show("  (generated FIRST, before any randomiser existed on the graph)")
show()


def variation(paths):
    """Mean per-pixel standard deviation across the dataset."""
    from PIL import Image
    arrs = [np.asarray(Image.open(p).convert("L"), dtype=np.float32)
            for p in paths if p.endswith(".png")][:N]
    if len(arrs) < 2:
        return None, len(arrs)
    stack = np.stack(arrs)
    return float(np.mean(np.std(stack, axis=0))), len(arrs)


v_fix, n_fix = variation([f for f in f_fix if "rgb" in f])
v_rand, n_rand = variation([f for f in f_rand if "rgb" in f])
show(f"  per-pixel std across the FIXED dataset    : {v_fix:.3f}  "
     f"({n_fix} images)")
show(f"  per-pixel std across the RANDOMISED dataset: {v_rand:.3f}  "
     f"({n_rand} images)")
if v_fix is not None and v_rand is not None and v_fix > 1e-9:
    show(f"  ratio: {v_rand / max(v_fix, 1e-9):.1f}x")
show()
if v_fix is not None and v_fix < 1.0:
    show("  THE FIXED DATASET IS ESSENTIALLY ONE IMAGE, REPEATED. Every file")
    show("  is a different file. Every one has a valid label. The count is")
    show(f"  {n_fix}, the directory looks exactly like a dataset, and there is")
    show("  almost nothing in it for a model to learn from.")
    show()
    show("  Nothing in the pipeline reports this. No error, no warning. The")
    show("  only way to catch it is to MEASURE the variation, which is why")
    show("  this number is in the lecture at all.")
else:
    show("  The fixed run varied more than expected -- report the real number")
    show("  rather than the tidy story.")
show()

show("=" * 68)
show("done")
show("=" * 68)

sim.close()
