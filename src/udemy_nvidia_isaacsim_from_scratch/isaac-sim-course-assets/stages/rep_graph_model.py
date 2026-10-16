# 10.2: Replicator's graph model.
#
# THE TRAP THIS LECTURE HAS TO AVOID. "Replicator is a graph" is the kind of
# sentence that sounds like an explanation and teaches nothing. Section 8.7
# already showed what a graph buys and costs, so this lecture does not re-teach
# OmniGraph. It answers the question a learner actually has after 10.1:
#
#   I wrote `with rep.trigger.on_frame(): ... rep.modify.pose(...)`. What did
#   that DO, and why did it keep happening after I stopped asking for it?
#
# 10.1 already paid for that question the hard way: the trigger from the
# randomised run stayed attached and silently randomised the "fixed" control
# run, and the two datasets came out identical in variation. That was not a bug
# in Replicator. It is the graph model working exactly as designed, and this
# lecture is where that becomes obvious rather than surprising.
#
# WHAT IS MEASURED:
#   1. What a `with` block actually creates -- the node count on the stage
#      before and after, so "it builds a graph" is a number rather than a claim.
#   2. That the graph PERSISTS: register a randomiser, then generate frames
#      without asking for it again, and watch it still fire.
#   3. That registering the SAME randomiser twice stacks it rather than
#      replacing it -- the reason a scene drifts further every run.
#   4. How to actually clear it, and proof that clearing worked.
#
# Everything drives rep.orchestrator.step() explicitly. run_until_complete()
# never returns headless -- 45 minutes, zero files (see rep_probe.py).
from isaacsim import SimulationApp

sim = SimulationApp({"headless": True})

import glob
import os
import shutil

import numpy as np
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.api import World
from pxr import Usd

OUT = "/workspace/out/s10_2"
N = 12
RES = (192, 192)


def show(l=""):
    print("R102> " + l, flush=True)


def graph_prims():
    """How many prims live under the Replicator graph right now."""
    stage = omni.usd.get_context().get_stage()
    n = 0
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        p = str(prim.GetPath())
        if "/Replicator" in p:
            n += 1
    return n


def files():
    return sorted(f for f in glob.glob(os.path.join(OUT, "**", "*"),
                                       recursive=True) if os.path.isfile(f))


def spread(paths):
    """Mean per-pixel std across a set of pngs -- how varied the batch is."""
    from PIL import Image
    a = [np.asarray(Image.open(p).convert("L"), dtype=np.float32)
         for p in paths if p.endswith(".png")]
    if len(a) < 2:
        return None
    return float(np.mean(np.std(np.stack(a), axis=0)))


def batch(tag):
    """Generate N frames into a fresh directory and report their spread."""
    shutil.rmtree(OUT, ignore_errors=True)
    os.makedirs(OUT, exist_ok=True)
    w = rep.WriterRegistry.get("BasicWriter")
    w.initialize(output_dir=OUT, rgb=True)
    w.attach([rp])
    for _ in range(N):
        rep.orchestrator.step(rt_subframes=1)
    w.detach()
    s = spread([f for f in files() if "rgb" in f])
    show(f"  {tag:34s} spread {s:7.3f}   ({len(files())} files)")
    return s


show("=" * 68)
show("10.2  REPLICATOR'S GRAPH MODEL -- what `with` actually builds")
show("=" * 68)
show()

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
world.reset()

cam = rep.create.camera(position=(2.6, 2.6, 1.9), look_at=(0, 0, 0.25))
rp = rep.create.render_product(cam, RES)
boxes = rep.create.cube(semantics=[("class", "box")], count=3,
                        position=(0, 0, 0.25), scale=0.3)

# ---------------------------------------------------------------------------
# 1. WHAT A `with` BLOCK CREATES
# ---------------------------------------------------------------------------
show("-" * 68)
show("1. WHAT THE `with` BLOCK BUILDS")
show("-" * 68)
before = graph_prims()
show(f"  prims under /Replicator before: {before}")

with rep.trigger.on_frame():
    with boxes:
        rep.modify.pose(
            position=rep.distribution.uniform((-0.8, -0.8, 0.15),
                                              (0.8, 0.8, 0.5)))
after = graph_prims()
show(f"  prims under /Replicator after : {after}   (+{after - before})")
show()
show("  That block did not randomise anything. It BUILT something: nodes on")
show("  the stage that will randomise, every time the graph is triggered.")
show("  The distinction matters because those nodes are still there when your")
show("  `with` block has long since finished executing.")
show()

# ---------------------------------------------------------------------------
# 2. IT PERSISTS
# ---------------------------------------------------------------------------
show("-" * 68)
show("2. IT KEEPS FIRING, WITHOUT BEING ASKED AGAIN")
show("-" * 68)
s1 = batch("first batch, right after the with")
s2 = batch("second batch, nothing re-registered")
show()
if s1 and s2 and abs(s2 - s1) / max(s1, 1e-9) < 0.5:
    show("  The second batch is as varied as the first, and nothing in this")
    show("  script asked for randomisation the second time. The graph is a")
    show("  standing instruction, not a function call.")
    show()
    show("  THIS IS WHAT BIT 10.1. The control run there -- the one that was")
    show("  supposed to have NO randomisation -- was generated after a trigger")
    show("  had already been registered, so it was randomised too, and the two")
    show("  datasets came out equally varied. The measurement caught it. The")
    show("  fix was to generate the control FIRST, before any trigger exists.")
else:
    show("  The two batches differ more than expected -- report the numbers")
    show("  rather than the expected story.")
show()

# ---------------------------------------------------------------------------
# 3. REGISTERING IT AGAIN STACKS IT
# ---------------------------------------------------------------------------
show("-" * 68)
show("3. REGISTERING IT TWICE DOES NOT REPLACE IT")
show("-" * 68)
n_before = graph_prims()
with rep.trigger.on_frame():
    with boxes:
        rep.modify.pose(
            position=rep.distribution.uniform((-0.8, -0.8, 0.15),
                                              (0.8, 0.8, 0.5)))
n_after = graph_prims()
show(f"  prims: {n_before} -> {n_after}   (+{n_after - n_before})")
s3 = batch("with the randomiser registered twice")
show()
show("  Running the same cell twice in a notebook is the everyday version of")
show("  this, and it is why a scene that looked right yesterday drifts today.")
show()

# ---------------------------------------------------------------------------
# 4. CLEARING IT, AND PROVING THE CLEAR WORKED
# ---------------------------------------------------------------------------
show("-" * 68)
show("4. CLEARING IT -- AND CHECKING")
show("-" * 68)
try:
    rep.orchestrator.stop()
except Exception:
    pass
stage = omni.usd.get_context().get_stage()
root = stage.GetPrimAtPath("/Replicator")
cleared = False
if root and root.IsValid():
    stage.RemovePrim("/Replicator")
    cleared = True
show(f"  removed /Replicator: {cleared}")
show(f"  prims under /Replicator now: {graph_prims()}")
show()
show("  A cleared graph is a claim, so check it: generate one more batch and")
show("  see whether the boxes still move.")
# the render product went with the graph, so rebuild the minimum to look
cam2 = rep.create.camera(position=(2.6, 2.6, 1.9), look_at=(0, 0, 0.25))
rp = rep.create.render_product(cam2, RES)
s4 = batch("after clearing the graph")
show()
if s4 is not None and s1 is not None:
    show(f"  spread {s1:.3f} with the randomiser, {s4:.3f} without: "
         f"{s1 / max(s4, 1e-9):.1f}x")
    if s4 < s1 * 0.5:
        show("  The clear worked, and now it is measured rather than assumed.")
    else:
        show("  The clear did NOT take. Report that -- a graph you believe you")
        show("  cleared and have not is worse than one you know is there.")
show()

show("=" * 68)
show("done")
show("=" * 68)

sim.close()
