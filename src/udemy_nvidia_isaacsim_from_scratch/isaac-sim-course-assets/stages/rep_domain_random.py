# 10.8: domain randomisation that actually helps.
#
# THE SECTION CLOSES BY JOINING TWO EARLIER MEASUREMENTS.
#   10.3 measured which knobs move the DATA: light +9.150, pose +5.455,
#        material +0.720, and the three do not add (0.74 of the sum).
#   10.7 measured which corruption breaks the MODEL: exposure x0.75 costs 9.0%,
#        while sensor noise costs 0.2% and defocus blur 0.5%.
#
# Those are different questions and the naive practice conflates them.
# "Randomise everything" spends effort on whatever is easiest to randomise.
# The measured practice is: randomise the thing your model is FRAGILE to.
#
# THE EXPERIMENT. Two datasets from the same scene, same size, same everything
# except one knob:
#   FIXED      lighting held constant
#   RANDOMISED lighting intensity varied across a wide range
# Train the same architecture on each, then test BOTH on the exposure
# perturbation that 10.7 showed is the one that hurts.
#
# THE PREDICTION, written down before running: the randomised model should lose
# less to exposure, because it has seen brightness vary. If it does not, this
# lecture says so -- domain randomisation is not a magic word, and a section
# that has caught its own artifacts four times is not going to stop now.
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
from pxr import Usd

N_TRAIN, N_TEST = 128, 40
RES = (160, 120)


def show(l=""):
    print("R108> " + l, flush=True)


def clear_graph():
    try:
        rep.orchestrator.stop()
    except Exception:
        pass
    st = omni.usd.get_context().get_stage()
    if st.GetPrimAtPath("/Replicator").IsValid():
        st.RemovePrim("/Replicator")


show("=" * 70)
show("10.8  DOMAIN RANDOMISATION THAT ACTUALLY HELPS")
show("=" * 70)
show()
show("  PREDICTION, written before running: the model trained with randomised")
show("  lighting should lose LESS to the exposure perturbation, because it has")
show("  seen brightness vary. If it does not, the lecture reports that.")
show()

root = get_assets_root_path()
open_stage(root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd")


def build(out_dir, randomise_light):
    """Generate a dataset. One knob differs between the two calls."""
    clear_graph()
    cam = rep.create.camera()
    rp = rep.create.render_product(cam, RES)
    props = rep.create.cube(semantics=[("class", "crate")], count=6,
                            position=(0, 0, 0.5), scale=0.6)
    lamp = rep.create.light(light_type="Sphere", intensity=6000,
                            position=(0.0, 0.0, 6.0))
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
                                                  (7.0, 13.0, 1.4)))
        if randomise_light:
            with lamp:
                rep.modify.attribute(
                    "inputs:intensity",
                    rep.distribution.uniform(1500, 20000))
    shutil.rmtree(out_dir, ignore_errors=True)
    os.makedirs(out_dir, exist_ok=True)
    w = rep.WriterRegistry.get("BasicWriter")
    w.initialize(output_dir=out_dir, rgb=True, bounding_box_2d_tight=True)
    w.attach([rp])
    for _ in range(3):
        rep.orchestrator.step(rt_subframes=1)
    shutil.rmtree(out_dir, ignore_errors=True)
    os.makedirs(out_dir, exist_ok=True)
    for _ in range(N_TRAIN + N_TEST):
        rep.orchestrator.step(rt_subframes=1)
    w.detach()


def load(d):
    from PIL import Image
    rg = sorted(glob.glob(os.path.join(d, "**", "rgb_*.png"), recursive=True))
    bb = sorted(glob.glob(os.path.join(d, "**",
                                       "bounding_box_2d_tight_*.npy"),
                          recursive=True))
    n = min(len(rg), len(bb))
    X = np.stack([np.asarray(Image.open(rg[i]).convert("L"),
                             dtype=np.float32) / 255.0 for i in range(n)])
    y = np.array([len(np.load(bb[i], allow_pickle=True))
                  for i in range(n)], dtype=np.float32)
    return X, y


import torch
import torch.nn as nn


def train_eval(X, y, tag):
    """Train, then report clean and exposure-perturbed error."""
    split = len(X) - N_TEST
    Xtr, ytr, Xte, yte = X[:split], y[:split], X[split:], y[split:]
    torch.manual_seed(0)
    net = nn.Sequential(
        nn.Conv2d(1, 8, 5, stride=2, padding=2), nn.ReLU(),
        nn.Conv2d(8, 16, 5, stride=2, padding=2), nn.ReLU(),
        nn.Conv2d(16, 32, 3, stride=2, padding=1), nn.ReLU(),
        nn.AdaptiveAvgPool2d(1), nn.Flatten(), nn.Linear(32, 1))
    opt = torch.optim.Adam(net.parameters(), lr=3e-3)
    lf = nn.L1Loss()
    Xt = torch.tensor(Xtr).unsqueeze(1)
    yt = torch.tensor(ytr).unsqueeze(1)
    for ep in range(300):
        perm = torch.randperm(len(Xt))
        for i in range(0, len(perm), 16):
            idx = perm[i:i + 16]
            opt.zero_grad()
            lf(net(Xt[idx]), yt[idx]).backward()
            opt.step()
    net.eval()

    def mae(arr):
        with torch.no_grad():
            p = net(torch.tensor(arr).unsqueeze(1)).squeeze(1).numpy()
        return float(np.mean(np.abs(p - yte)))

    floor = float(np.mean(np.abs(yte - ytr.mean())))
    clean = mae(Xte)
    dark = mae(np.clip(Xte * 0.75, 0, 1))
    bright = mae(np.clip(Xte * 1.30, 0, 1))
    skill = (floor - clean) / floor
    show(f"  {tag}")
    show(f"    floor {floor:6.2f}   clean {clean:6.2f}   "
         f"skill {skill * 100:5.1f}%")
    show(f"    exposure x0.75 {dark:6.2f}  ({(dark - clean) / clean * 100:+6.1f}%)")
    show(f"    exposure x1.30 {bright:6.2f}  ({(bright - clean) / clean * 100:+6.1f}%)")
    return dict(floor=floor, clean=clean, dark=dark, bright=bright, skill=skill)


show("-" * 70)
show("1. TWO DATASETS, ONE KNOB DIFFERENT")
show("-" * 70)
t0 = time.perf_counter()
build("/workspace/out/s10_8_fixed", randomise_light=False)
Xf, yf = load("/workspace/out/s10_8_fixed")
build("/workspace/out/s10_8_rand", randomise_light=True)
Xr, yr = load("/workspace/out/s10_8_rand")
show(f"  generated {len(Xf)} + {len(Xr)} frames in {time.perf_counter() - t0:.0f} s")
show(f"  fixed-light pixel std across the set     : {Xf.std():.4f}")
show(f"  randomised-light pixel std across the set: {Xr.std():.4f}")
show()

show("-" * 70)
show("2. THE SAME ARCHITECTURE, TRAINED ON EACH")
show("-" * 70)
F = train_eval(Xf, yf, "trained on FIXED lighting")
show()
R = train_eval(Xr, yr, "trained on RANDOMISED lighting")
show()

show("-" * 70)
show("3. DID IT HELP?")
show("-" * 70)
f_loss = (F["dark"] - F["clean"]) / F["clean"] * 100
r_loss = (R["dark"] - R["clean"]) / R["clean"] * 100
show(f"  loss to exposure x0.75:  fixed {f_loss:+.1f}%   "
     f"randomised {r_loss:+.1f}%")
show()
if min(F["skill"], R["skill"]) < 0.10:
    show("  ONE OF THESE MODELS BARELY LEARNED, so the comparison cannot be")
    show("  trusted -- 10.7's lesson exactly. Report that rather than a")
    show("  conclusion the experiment cannot support.")
elif r_loss < f_loss - 1.0:
    show("  THE PREDICTION HELD. Randomising the knob the model was fragile to")
    show("  made it more robust to exactly that corruption. Note what this is")
    show("  NOT: it is not a general improvement, it is robustness to the one")
    show("  thing that was randomised.")
else:
    show("  THE PREDICTION DID NOT HOLD, and that is the result. Randomising")
    show("  the lighting did not buy robustness to exposure here. Possible")
    show("  reasons worth testing rather than asserting: the randomisation")
    show("  range may not cover the perturbation, the model may be reading")
    show("  something other than brightness, or 128 frames may be too few to")
    show("  learn invariance from. Domain randomisation is a technique with")
    show("  conditions, not a magic word.")
show()
show("  AND THE COST, which nobody quotes: randomising lighting changed the")
show(f"  dataset's own pixel spread from {Xf.std():.4f} to {Xr.std():.4f}. More")
show("  variation is not automatically more signal -- 10.3 measured that the")
show("  knobs overlap and do not add, and 10.1 measured that variation without")
show("  relevance is just noise with a bigger file size.")
show()
show("=" * 70)
show("done")
show("=" * 70)

sim.close()
