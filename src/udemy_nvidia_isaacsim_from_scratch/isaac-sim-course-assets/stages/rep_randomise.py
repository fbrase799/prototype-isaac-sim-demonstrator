# 10.3: randomising pose, light and material.
#
# 10.1 showed that a dataset without variation is worthless. 10.2 showed how the
# randomiser gets onto the stage. This lecture is about WHICH variation to add,
# and it is the one where a course can quietly go wrong, because "randomise
# everything" sounds like good advice and is not.
#
# THE QUESTION THIS ANSWERS: pose, lighting and material are the three knobs
# everyone reaches for. Do they contribute equally? Measure them separately and
# find out, rather than asserting the usual "all three matter" line.
#
# WHAT IS MEASURED, each in isolation, same scene, same frame count:
#   baseline   nothing randomised
#   pose       objects move
#   light      intensity and position move
#   material   colour/roughness move
#   all three  together
# reported as per-pixel spread across the dataset, so the contributions are
# directly comparable, plus the render cost of each so the price is visible.
#
# THE TRAP FROM 10.2 IS LOAD-BEARING HERE: a trigger persists, so every one of
# these has to be measured with a CLEAN graph or the later ones inherit the
# earlier ones. Each condition clears /Replicator first and the clear is
# checked, not assumed.
from isaacsim import SimulationApp

sim = SimulationApp({"headless": True})

import glob
import os
import shutil
import time

import numpy as np
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.api import World
from pxr import Usd

OUT = "/workspace/out/s10_3"
N = 16
RES = (192, 192)


def show(l=""):
    print("R103> " + l, flush=True)


def clear_graph():
    """Remove the whole Replicator graph. 10.2's lesson, applied every time."""
    try:
        rep.orchestrator.stop()
    except Exception:
        pass
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath("/Replicator").IsValid():
        stage.RemovePrim("/Replicator")


def n_graph():
    stage = omni.usd.get_context().get_stage()
    return sum(1 for p in Usd.PrimRange(stage.GetPseudoRoot())
               if "/Replicator" in str(p.GetPath()))


def spread(paths):
    from PIL import Image
    a = [np.asarray(Image.open(p).convert("L"), dtype=np.float32)
         for p in paths if p.endswith(".png")]
    if len(a) < 2:
        return None
    return float(np.mean(np.std(np.stack(a), axis=0)))


world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
world.reset()

show("=" * 70)
show("10.3  RANDOMISING POSE, LIGHT AND MATERIAL -- which one earns its place?")
show("=" * 70)
show()

results = []


def condition(name, pose=False, light=False, material=False):
    """Build ONE condition on a clean graph and measure it."""
    clear_graph()
    # every condition starts from nothing, and that is checked rather than
    # assumed -- 10.2 showed what inheriting a stale trigger does to a control
    assert n_graph() <= 2, f"graph not clean before {name}: {n_graph()} prims"

    cam = rep.create.camera(position=(2.6, 2.6, 1.9), look_at=(0, 0, 0.25))
    rp = rep.create.render_product(cam, RES)
    boxes = rep.create.cube(semantics=[("class", "box")], count=3,
                            position=(0, 0, 0.25), scale=0.3)
    lamp = rep.create.light(light_type="Sphere", intensity=6000,
                            position=(1.5, 1.5, 3.0))

    with rep.trigger.on_frame():
        if pose:
            with boxes:
                rep.modify.pose(
                    position=rep.distribution.uniform((-0.8, -0.8, 0.15),
                                                      (0.8, 0.8, 0.5)),
                    rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360)))
        if light:
            with lamp:
                rep.modify.attribute(
                    "inputs:intensity",
                    rep.distribution.uniform(1200, 14000))
                rep.modify.pose(
                    position=rep.distribution.uniform((-2.5, -2.5, 1.5),
                                                      (2.5, 2.5, 4.0)))
        if material:
            with boxes:
                rep.randomizer.color(
                    colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))

    shutil.rmtree(OUT, ignore_errors=True)
    os.makedirs(OUT, exist_ok=True)
    w = rep.WriterRegistry.get("BasicWriter")
    w.initialize(output_dir=OUT, rgb=True)
    w.attach([rp])
    t0 = time.perf_counter()
    for _ in range(N):
        rep.orchestrator.step(rt_subframes=1)
    dt = time.perf_counter() - t0
    w.detach()

    files = sorted(f for f in glob.glob(os.path.join(OUT, "**", "*"),
                                        recursive=True)
                   if os.path.isfile(f) and f.endswith(".png"))
    s = spread(files)
    results.append((name, s, dt))
    show(f"  {name:26s} spread {s:7.3f}   {dt / N * 1000:6.1f} ms/frame")
    return s


show("-" * 70)
show("EACH KNOB ON ITS OWN, SAME SCENE, SAME 16 FRAMES")
show("-" * 70)
base = condition("baseline (nothing)")
p_only = condition("pose only", pose=True)
l_only = condition("light only", light=True)
m_only = condition("material only", material=True)
all3 = condition("all three", pose=True, light=True, material=True)
show()

# ---------------------------------------------------------------------------
# WHAT THE NUMBERS SAY
# ---------------------------------------------------------------------------
show("-" * 70)
show("WHAT THE NUMBERS SAY")
show("-" * 70)
if base and base > 1e-9:
    for name, s, _ in results[1:]:
        show(f"  {name:26s} {s / base:6.2f}x the baseline")
    show()
    contrib = sorted(((s - base, n) for n, s, _ in results[1:-1]), reverse=True)
    show("  ranked by what each ADDS over the baseline:")
    for d, n in contrib:
        show(f"    {n:24s} +{d:6.3f}")
    show()
    biggest = contrib[0][1]
    smallest = contrib[-1][1]
    show(f"  The biggest single contributor here is {biggest.upper()}, and the")
    show(f"  smallest is {smallest.upper()}. That ordering is a property of THIS")
    show("  scene, not a law: three boxes on a plain floor seen from one fixed")
    show("  camera. Move the camera instead and the ranking changes.")
    show()
    # is the whole the sum of its parts?
    sum_parts = sum(d for d, _ in contrib) + (results[3][1] - base)
    got = all3 - base
    show(f"  sum of the three separate contributions: {sum_parts:6.3f}")
    show(f"  measured contribution of all three:      {got:6.3f}")
    show(f"  ratio {got / max(sum_parts, 1e-9):.2f}")
    if got < sum_parts * 0.9:
        show("  They do NOT simply add. The knobs overlap -- moving an object")
        show("  and moving the light both change the same pixels, so stacking")
        show("  randomisers buys less than the arithmetic suggests.")
    else:
        show("  They add roughly linearly here; report that rather than the")
        show("  expected overlap story.")
else:
    show("  baseline spread was ~0, so the ratios are undefined -- report the")
    show("  raw numbers rather than dividing by nothing.")
show()
show("=" * 70)
show("done")
show("=" * 70)

sim.close()
