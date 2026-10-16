# Section 10 feasibility, attempt 2 -- the SMALLEST possible Replicator job.
#
# Attempt 1 (rep_probe.py) imported fine and built the randomiser fine, then sat
# in rep.orchestrator.run_until_complete() for 45+ minutes at ~200% CPU and 42%
# GPU and wrote ZERO files. The output directory was created; nothing landed in
# it. The known root-owned-cache trap is NOT the cause here -- /isaac-sim/.cache
# and /isaac-sim/kit/cache are both owned by isaac-sim.
#
# So this attempt changes exactly three things, so that whatever happens is
# attributable:
#   1. rep.orchestrator.step() in an EXPLICIT loop instead of the blocking
#      run_until_complete(), which may simply never satisfy its condition here.
#   2. one frame, 128x128, rgb only -- no bounding-box annotator, in case the
#      annotator is what stalls.
#   3. a file count printed after EVERY step, so a partial success is visible
#      instead of being hidden behind a call that never returns.
from isaacsim import SimulationApp

sim = SimulationApp({"headless": True})

import glob
import os
import time

import omni.replicator.core as rep
from isaacsim.core.api import World

OUT = "/workspace/out/rep_probe2"


def show(l=""):
    print("RP2> " + l, flush=True)


def nfiles():
    return len([f for f in glob.glob(os.path.join(OUT, "**", "*"),
                                     recursive=True) if os.path.isfile(f)])


show("=" * 66)
show("REPLICATOR, SMALLEST POSSIBLE JOB -- explicit steps, rgb only")
show("=" * 66)

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
world.reset()

cam = rep.create.camera(position=(2.5, 2.5, 2.0), look_at=(0, 0, 0.2))
cube = rep.create.cube(semantics=[("class", "box")], position=(0, 0, 0.2),
                       scale=0.3)
rp = rep.create.render_product(cam, (128, 128))

os.makedirs(OUT, exist_ok=True)
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=OUT, rgb=True)      # rgb ONLY, no annotators
writer.attach([rp])
show(f"writer attached, output {OUT}")
show(f"files before: {nfiles()}")
show()

# Explicit stepping, with a wall-clock budget. If a single frame cannot be
# produced in 240 s on a 128x128 render, the answer is no and the section has
# to be replanned around it.
BUDGET = 240.0
t0 = time.perf_counter()
for i in range(4):
    el = time.perf_counter() - t0
    if el > BUDGET:
        show(f"budget spent at step {i}; stopping")
        break
    s0 = time.perf_counter()
    try:
        rep.orchestrator.step(rt_subframes=1)
    except Exception as e:
        show(f"step {i} raised: {type(e).__name__}: {e}")
        break
    show(f"  step {i}: {time.perf_counter() - s0:6.2f} s   files now {nfiles()}")

show()
n = nfiles()
show(f"FILES WRITTEN: {n}")
if n:
    for f in sorted(glob.glob(os.path.join(OUT, "**", "*"), recursive=True))[:8]:
        if os.path.isfile(f):
            show(f"  {os.path.getsize(f):9d}  {f}")
    show()
    show("VERDICT: Replicator CAN write a dataset here with explicit stepping.")
    show("Section 10 is recordable; use rep.orchestrator.step(), never")
    show("run_until_complete(), and budget the measured per-step cost.")
else:
    show()
    show("VERDICT: still nothing on disk. Replicator renders (CPU and GPU are")
    show("busy) but does not deliver files in this container. Section 10 needs")
    show("replanning -- do NOT write eight lectures against it.")

sim.close()
