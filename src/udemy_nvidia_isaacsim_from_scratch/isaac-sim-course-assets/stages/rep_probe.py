# Section 10 feasibility probe -- does Replicator actually WRITE a dataset here?
#
# WHY THIS EXISTS. The Section 9 probe cost an hour and saved nine lectures by
# answering "does the thing work in this container" before any were written.
# Section 10 has the same exposure: eight lectures that assume Replicator can
# randomise a scene, render it, and write annotated files to disk. If any of
# that fails headless, the section has to be replanned, and it is much cheaper
# to know now.
#
# WHAT IS CHECKED, cheapest first:
#   1. Does omni.replicator.core import and enable?
#   2. Can a randomiser graph be created? (8.7's lesson: existing != working)
#   3. Does a writer actually produce FILES ON DISK, with real annotations?
#   4. Do the randomisations differ between frames, or is every frame identical?
#      (A dataset of 100 identical images passes every "it ran" check and is
#      worthless -- that is the Replicator version of a frozen mp4.)
from isaacsim import SimulationApp

sim = SimulationApp({"headless": True})

import os
import glob
import numpy as np


def show(l=""):
    print("REP> " + l, flush=True)


OUT = "/workspace/out/rep_probe"
show("=" * 70)
show("SECTION 10 FEASIBILITY -- does Replicator write a real dataset here?")
show("=" * 70)
show()

# ---------------------------------------------------------------------------
# 1. IMPORT
# ---------------------------------------------------------------------------
show("-" * 70)
show("1. import omni.replicator.core")
show("-" * 70)
try:
    import omni.replicator.core as rep
    show(f"  imported, version attr: {getattr(rep, '__version__', 'n/a')}")
except Exception as e:
    show(f"  FAILED: {type(e).__name__}: {e}")
    show("  Section 10 cannot be recorded here. Stop and replan.")
    sim.close()
    raise SystemExit(1)
show()

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
world.reset()

# ---------------------------------------------------------------------------
# 2. A SCENE WITH SOMETHING TO RANDOMISE
# Deliberately tiny: three cubes and a light. The lecture's scene comes later;
# this only has to prove the machinery moves.
# ---------------------------------------------------------------------------
show("-" * 70)
show("2. build a randomiser")
show("-" * 70)
try:
    cam = rep.create.camera(position=(3.0, 3.0, 2.5), look_at=(0, 0, 0.3))
    cubes = rep.create.cube(
        semantics=[("class", "box")],
        position=rep.distribution.uniform((-1, -1, 0.2), (1, 1, 0.6)),
        scale=rep.distribution.uniform(0.15, 0.35),
        count=3,
    )
    light = rep.create.light(
        light_type="Sphere",
        intensity=rep.distribution.uniform(2000, 12000),
        position=rep.distribution.uniform((-2, -2, 2), (2, 2, 4)),
    )
    with rep.trigger.on_frame(num_frames=6):
        with cubes:
            rep.modify.pose(
                position=rep.distribution.uniform((-1, -1, 0.2), (1, 1, 0.6)))
        with light:
            rep.modify.attribute(
                "inputs:intensity", rep.distribution.uniform(2000, 12000))
    show("  randomiser graph created: 3 cubes + 1 light, 6 frames")
except Exception as e:
    show(f"  FAILED: {type(e).__name__}: {e}")
    sim.close()
    raise SystemExit(1)
show()

# ---------------------------------------------------------------------------
# 3. A WRITER, AND FILES ON DISK
# The question is not "did it run" but "is there a dataset".
# ---------------------------------------------------------------------------
show("-" * 70)
show("3. attach a writer and run")
show("-" * 70)
try:
    os.makedirs(OUT, exist_ok=True)
    rp = rep.create.render_product(cam, (320, 240))
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=OUT, rgb=True, bounding_box_2d_tight=True)
    writer.attach([rp])
    rep.orchestrator.run_until_complete()
    show("  orchestrator finished")
except Exception as e:
    show(f"  FAILED: {type(e).__name__}: {e}")
    sim.close()
    raise SystemExit(1)

files = sorted(glob.glob(os.path.join(OUT, "**", "*"), recursive=True))
files = [f for f in files if os.path.isfile(f)]
show(f"  {len(files)} files written under {OUT}")
by_ext = {}
for f in files:
    by_ext.setdefault(os.path.splitext(f)[1], []).append(f)
for ext, fs in sorted(by_ext.items()):
    show(f"    {ext or '(none)':10s} x{len(fs):3d}   e.g. {os.path.basename(fs[0])}")
show()

# ---------------------------------------------------------------------------
# 4. ARE THE FRAMES ACTUALLY DIFFERENT?
# A hundred identical images passes every "did it run" check and is worthless.
# This is the Replicator version of the frozen-mp4 trap from the video gates.
# ---------------------------------------------------------------------------
show("-" * 70)
show("4. do the frames differ?")
show("-" * 70)
pngs = sorted(by_ext.get(".png", []))
if len(pngs) < 2:
    show(f"  only {len(pngs)} png(s) -- cannot compare. Treat as a FAILURE:")
    show("  a dataset you cannot check for variation is not a dataset.")
else:
    try:
        from PIL import Image
        arrs = [np.asarray(Image.open(p).convert("L"), dtype=np.float32)
                for p in pngs[:6]]
        base = arrs[0]
        diffs = [float(np.mean(np.abs(a - base))) for a in arrs[1:]]
        for p, d in zip(pngs[1:6], diffs):
            show(f"    {os.path.basename(p):32s} mean |Δ| vs first = {d:7.3f}")
        moved = sum(1 for d in diffs if d > 1.0)
        show(f"  {moved} of {len(diffs)} frames differ materially from the first")
        if moved == 0:
            show()
            show("  EVERY FRAME IS THE SAME. The randomisation is not taking, and")
            show("  a dataset like this trains nothing. This is the exact trap")
            show("  10.1 has to warn about.")
    except Exception as e:
        show(f"  comparison failed: {type(e).__name__}: {e}")
show()

show("=" * 70)
show("done")
show("=" * 70)

sim.close()
