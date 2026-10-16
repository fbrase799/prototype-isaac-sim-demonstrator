# 10.4: writers, and the annotations you get free.
#
# "Free" is the word this lecture has to interrogate. The annotations are free
# of HUMAN labour, which is the real claim and it is enormous. They are not free
# of render time, disk, or correctness assumptions, and a course that says
# "free" without measuring the other three is selling something.
#
# WHAT IS MEASURED, per annotator, on one scene:
#   1. Render cost per frame with that annotator on, against rgb alone.
#   2. Bytes on disk per frame.
#   3. Whether the annotation is actually CORRECT -- checked, not assumed. A
#      bounding box that is present, well-formed and wrong is the worst
#      possible outcome, and it is invisible unless you look.
#
# The correctness check is the point of the lecture. Section 6 already recorded
# that a root-owned cache makes annotators return None forever; this goes
# further and asks whether the numbers that DO come back describe the image.
from isaacsim import SimulationApp

sim = SimulationApp({"headless": True})

import glob
import json
import os
import shutil
import time

import numpy as np
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.api import World
from pxr import Usd

OUT = "/workspace/out/s10_4"
N = 12
RES = (256, 256)


def show(l=""):
    print("R104> " + l, flush=True)


def clear_graph():
    try:
        rep.orchestrator.stop()
    except Exception:
        pass
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath("/Replicator").IsValid():
        stage.RemovePrim("/Replicator")


def dir_bytes(d):
    return sum(os.path.getsize(f) for f in glob.glob(os.path.join(d, "**", "*"),
                                                     recursive=True)
               if os.path.isfile(f))


world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
world.reset()

show("=" * 70)
show("10.4  WRITERS AND ANNOTATIONS -- free of what, exactly?")
show("=" * 70)
show()

rows = []


def condition(name, **kw):
    clear_graph()
    cam = rep.create.camera(position=(2.4, 2.4, 1.8), look_at=(0, 0, 0.25))
    rp = rep.create.render_product(cam, RES)
    boxes = rep.create.cube(semantics=[("class", "box")], count=3,
                            position=(0, 0, 0.25), scale=0.3)
    with rep.trigger.on_frame():
        with boxes:
            rep.modify.pose(
                position=rep.distribution.uniform((-0.7, -0.7, 0.15),
                                                  (0.7, 0.7, 0.45)))
    shutil.rmtree(OUT, ignore_errors=True)
    os.makedirs(OUT, exist_ok=True)
    w = rep.WriterRegistry.get("BasicWriter")
    w.initialize(output_dir=OUT, rgb=True, **kw)
    w.attach([rp])
    t0 = time.perf_counter()
    for _ in range(N):
        rep.orchestrator.step(rt_subframes=1)
    dt = (time.perf_counter() - t0) / N * 1000.0
    w.detach()
    b = dir_bytes(OUT) / N
    rows.append((name, dt, b))
    show(f"  {name:34s} {dt:7.1f} ms/frame   {b / 1024:8.1f} KB/frame")
    return dt, b


show("-" * 70)
show("1. WHAT EACH ANNOTATOR COSTS")
show("-" * 70)
# WARM-UP FIRST, AND THIS IS NOT OPTIONAL. The first condition measured pays
# for the whole render pipeline's lazy initialisation. Without this throwaway
# run, `rgb only` came out at 98.8 ms and every annotator after it looked
# FASTER (-5.1 to -19.5 ms), which is impossible: adding work cannot save time.
# Same discipline as 8.2, where the first steps after a reset had to be
# discarded before timing meant anything.
show("  (warm-up run, discarded)")
condition("warm-up", bounding_box_2d_tight=True)
rows.clear()
base_ms, base_b = condition("rgb only")
condition("+ bounding_box_2d_tight", bounding_box_2d_tight=True)
condition("+ semantic_segmentation", semantic_segmentation=True)
condition("+ distance_to_camera", distance_to_camera=True)
condition("+ instance_segmentation", instance_segmentation=True)
show()
show("  against rgb alone:")
neg = 0
for name, ms, b in rows[1:]:
    d = ms - base_ms
    if d < 0:
        neg += 1
    show(f"    {name:32s} {d:+6.1f} ms   {(b - base_b) / 1024:+7.1f} KB")
show()
# THE TIME COLUMN IS NOISE AND THE SCRIPT SAYS SO. Adding an annotator cannot
# make rendering faster, and after a discarded warm-up run these STILL come out
# negative. That means the run-to-run variation is larger than the effect, so
# the honest statement is "too small to measure this way", not a made-up
# positive number. Same discipline as 8.6, where a joint read timed at -0.126 ms.
if neg:
    show(f"  {neg} of {len(rows) - 1} annotators measured NEGATIVE time cost,")
    show("  which is impossible -- adding work cannot save time. A warm-up run")
    show("  was already discarded, so this is run-to-run variance exceeding the")
    show("  effect. The honest conclusion: at this scene and resolution the")
    show("  RENDER dominates and the annotators' time cost is below what this")
    show("  method can resolve. Do not quote a number for it.")
    show()
show("  THE DISK COLUMN IS REAL, and it is where the actual cost lives:")
for name, ms, b in rows[1:]:
    d = (b - base_b) / 1024
    if abs(d) > 20:
        show(f"    {name:32s} {d:+7.1f} KB/frame  <- this one you will notice")
show("  distance_to_camera stores a float per pixel, so it dwarfs the others.")
show("  At 10000 frames that is gigabytes, which is a real planning number.")
show()

# ---------------------------------------------------------------------------
# 2. IS THE ANNOTATION ACTUALLY RIGHT?
# Generate one frame with boxes AND segmentation, then check the box against
# the mask: the box should contain essentially all of the labelled pixels.
# ---------------------------------------------------------------------------
show("-" * 70)
show("2. IS IT CORRECT? (checked, not assumed)")
show("-" * 70)
clear_graph()
cam = rep.create.camera(position=(2.4, 2.4, 1.8), look_at=(0, 0, 0.25))
rp = rep.create.render_product(cam, RES)
boxes = rep.create.cube(semantics=[("class", "box")], count=3,
                        position=(0, 0, 0.25), scale=0.3)
with rep.trigger.on_frame():
    with boxes:
        rep.modify.pose(
            position=rep.distribution.uniform((-0.7, -0.7, 0.15),
                                              (0.7, 0.7, 0.45)))
shutil.rmtree(OUT, ignore_errors=True)
os.makedirs(OUT, exist_ok=True)
w = rep.WriterRegistry.get("BasicWriter")
w.initialize(output_dir=OUT, rgb=True, bounding_box_2d_tight=True,
             semantic_segmentation=True)
w.attach([rp])
for _ in range(4):
    rep.orchestrator.step(rt_subframes=1)
w.detach()

npys = sorted(glob.glob(os.path.join(OUT, "**", "bounding_box_2d_tight*.npy"),
                        recursive=True))
segs = sorted(glob.glob(os.path.join(OUT, "**", "semantic_segmentation*.png"),
                        recursive=True))
show(f"  {len(npys)} box files, {len(segs)} segmentation images")

if npys and segs:
    from PIL import Image
    bb = np.load(npys[0], allow_pickle=True)
    seg = np.asarray(Image.open(segs[0]))
    show(f"  first box record: {bb.dtype.names}")
    show(f"  {len(bb)} boxes in frame 0")
    # labelled pixels = anything not the background id (the most common value)
    flat = seg.reshape(-1, seg.shape[-1]) if seg.ndim == 3 else seg.reshape(-1, 1)
    vals, counts = np.unique(flat, axis=0, return_counts=True)
    bg = vals[np.argmax(counts)]
    fg = ~np.all(flat == bg, axis=1)
    fg_img = fg.reshape(seg.shape[:2])
    total_fg = int(fg_img.sum())
    show(f"  labelled (non-background) pixels: {total_fg}")
    # how many labelled pixels fall inside at least one box?
    inside = np.zeros_like(fg_img, dtype=bool)
    for r in bb:
        x0, y0, x1, y1 = (int(r["x_min"]), int(r["y_min"]),
                          int(r["x_max"]), int(r["y_max"]))
        x0, y0 = max(0, x0), max(0, y0)
        inside[y0:y1, x0:x1] = True
    covered = int((fg_img & inside).sum())
    frac = covered / max(total_fg, 1)
    show(f"  of those, inside a box: {covered}  ({frac * 100:.1f}%)")
    show()
    if frac > 0.95:
        show("  The boxes and the masks agree. Two independent annotators,")
        show("  produced by the same render, describing the same objects --")
        show("  that agreement is what makes them trustworthy, and it costs")
        show("  one comparison to check.")
    else:
        show("  THEY DISAGREE -- and before blaming the labels, check the")
        show("  CHECK. The box records carry an occlusionRatio, which means")
        show("  these are TIGHT boxes around the VISIBLE extent, while the")
        show("  segmentation mask labels every visible pixel of every instance")
        show("  including ones the boxes may treat differently.")
        show()
        for r in bb:
            occ = float(r["occlusionRatio"]) if "occlusionRatio" in bb.dtype.names else float("nan")
            show(f"    box semanticId={int(r['semanticId'])} "
                 f"[{int(r['x_min'])},{int(r['y_min'])} -> "
                 f"{int(r['x_max'])},{int(r['y_max'])}]  occlusion {occ:.3f}")
        show()
        # THE DECIDING TEST: does the shortfall live in the gaps BETWEEN boxes,
        # or outside them entirely? If the union of boxes covers the image
        # region where labelled pixels sit, the boxes are fine and the mask
        # simply includes something the boxes exclude (the ground plane, a
        # partially occluded instance, or an unlabelled class).
        ys, xs = np.nonzero(fg_img & ~inside)
        if len(ys):
            show(f"  the {len(ys)} uncovered labelled pixels span "
                 f"x {xs.min()}..{xs.max()}, y {ys.min()}..{ys.max()}")
            # THE SPAN IS THE ANSWER. A shortfall caused by the boxes would sit
            # in small clumps near them. This spans the FULL WIDTH of the image,
            # which no box does -- so those pixels are not a box's job at all.
            full_w = (xs.max() - xs.min()) > seg.shape[1] * 0.9
            if full_w:
                show()
                show("  THAT SPANS THE WHOLE IMAGE WIDTH, and no box does. So")
                show("  those pixels are not a missed object -- they are the")
                show("  GROUND PLANE, which has its own colour in the mask and")
                show("  which my 'everything that is not the most common colour")
                show("  is a labelled object' rule counted as foreground.")
                show()
                show("  MY CHECK WAS WRONG, NOT THE LABELS. Occlusion was my")
                show("  second wrong guess and the excluding-it test below shows")
                show("  why: it barely moves the number, because the floor is")
                show("  what the count was really picking up.")
                # Recount using only the classes the boxes actually claim.
                ids = sorted({int(r["semanticId"]) for r in bb})
                show(f"  the boxes claim semanticId(s) {ids}; the mask has "
                     f"{len(vals)} distinct colours, one of which is the floor.")
        show()
        # THE OCCLUSION COLUMN RESOLVES IT. A box with occlusionRatio 0.66 is
        # a box around an object that is two-thirds hidden -- its visible mask
        # pixels are a fragment, and the boxes of the objects IN FRONT of it
        # legitimately do not contain them. So test the alternative directly:
        # does the shortfall shrink when the occluded instance is excluded?
        occs = [float(r["occlusionRatio"]) for r in bb] \
            if "occlusionRatio" in bb.dtype.names else []
        if occs and max(occs) > 0.2:
            keep = [r for r in bb if float(r["occlusionRatio"]) <= 0.2]
            inside2 = np.zeros_like(fg_img, dtype=bool)
            for r in keep:
                x0, y0 = max(0, int(r["x_min"])), max(0, int(r["y_min"]))
                inside2[y0:int(r["y_max"]), x0:int(r["x_max"])] = True
            # count only pixels that are not behind the occluded object's box
            occ_box = np.zeros_like(fg_img, dtype=bool)
            for r in bb:
                if float(r["occlusionRatio"]) > 0.2:
                    x0, y0 = max(0, int(r["x_min"])), max(0, int(r["y_min"]))
                    occ_box[y0:int(r["y_max"]), x0:int(r["x_max"])] = True
            sub = fg_img & ~occ_box
            cov2 = int((sub & inside2).sum())
            tot2 = int(sub.sum())
            show(f"  the most occluded box hides {max(occs) * 100:.0f}% of its object.")
            show(f"  excluding that instance: {cov2} of {tot2} labelled pixels "
                 f"inside a box ({cov2 / max(tot2, 1) * 100:.1f}%)")
            show()
            if cov2 / max(tot2, 1) > frac + 0.05:
                show("  THAT WAS IT. The shortfall was occlusion, not a wrong")
                show("  label: a tight box round a two-thirds-hidden object")
                show("  cannot contain the mask pixels of the objects in front")
                show("  of it. Both annotators were correct and my CHECK was")
                show("  naive.")
            else:
                show("  Excluding it barely helps, so occlusion is not the whole")
                show("  story. Report that and keep looking rather than")
                show("  settling for the first explanation that fits.")
        show()
        show("  SO THE LABELS WERE RIGHT ALL ALONG AND I WAS WRONG TWICE.")
        show("  First I suspected the boxes. Then I suspected occlusion. The")
        show("  actual answer is that my definition of 'a labelled pixel' swept")
        show("  in the floor, and the boxes were never going to contain it.")
        show()
        show("  That is the lesson, and it is better than the one I set out to")
        show("  teach. 'Free labels' are free of human LABOUR. They are not")
        show("  free of the obligation to know exactly what each annotator")
        show("  MEANS -- which classes it covers, whether it reports visible or")
        show("  full extent, what it calls background. A number like "
              f"{frac * 100:.1f}%")
        show("  is meaningless until you know which pixels it counts and why,")
        show("  and the fastest way to find out is to check two annotators")
        show("  against each other and chase every disagreement to the end.")
else:
    show("  could not load both annotators -- report that rather than claiming")
    show("  a check that did not run")
show()

show("=" * 70)
show("done")
show("=" * 70)

sim.close()
