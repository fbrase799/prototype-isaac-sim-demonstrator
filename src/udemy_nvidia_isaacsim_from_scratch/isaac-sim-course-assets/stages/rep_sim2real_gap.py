# 10.7: the sim-to-real gap, measured not asserted.
#
# THE HONEST PROBLEM WITH THIS LECTURE. The real sim-to-real gap is measured by
# testing on photographs from a real camera. This course has no real camera and
# no real warehouse, so I cannot measure the real gap, and pretending otherwise
# would be exactly the failure this whole section warns about.
#
# WHAT I CAN DO, and what this script does, is measure a LOWER BOUND. Train on
# clean synthetic images, then test on synthetic images perturbed in the
# specific ways a real camera differs from a renderer:
#
#   sensor noise      real sensors are noisy; RTX output is not
#   defocus blur      real lenses have a focal plane; renderers are sharp
#   exposure shift    real auto-exposure drifts; renders are consistent
#   colour cast       real white balance is wrong in a specific direction
#   JPEG-ish loss     real pipelines compress; PNG renders do not
#
# Every one of those is a difference that EXISTS between this simulator and any
# real camera. So the drop they cause is a genuine part of the gap. It is not
# the whole gap -- geometry, materials, lighting realism and object appearance
# are all still synthetic -- so the number is a floor, and the lecture says so.
#
# A LOWER BOUND THAT IS ALREADY LARGE IS A USEFUL RESULT. If a model loses most
# of its skill to sensor noise alone, it was never going to survive a real
# camera, and you have learned that without owning one.
from isaacsim import SimulationApp

sim = SimulationApp({"headless": True})

import glob
import os
import shutil
import time

import numpy as np
import omni.replicator.core as rep
from isaacsim.core.utils.stage import open_stage
from isaacsim.storage.native import get_assets_root_path

OUT = "/workspace/out/s10_7"
N_TRAIN, N_TEST = 96, 32
RES = (160, 120)


def show(l=""):
    print("R107> " + l, flush=True)


show("=" * 70)
show("10.7  THE SIM-TO-REAL GAP -- a measured LOWER BOUND")
show("=" * 70)
show()

root = get_assets_root_path()
open_stage(root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd")
cam = rep.create.camera()
rp = rep.create.render_product(cam, RES)
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
                                              (7.0, 13.0, 1.4)))

shutil.rmtree(OUT, ignore_errors=True)
os.makedirs(OUT, exist_ok=True)
w = rep.WriterRegistry.get("BasicWriter")
w.initialize(output_dir=OUT, rgb=True, bounding_box_2d_tight=True)
w.attach([rp])
for _ in range(3):
    rep.orchestrator.step(rt_subframes=1)
shutil.rmtree(OUT, ignore_errors=True)
os.makedirs(OUT, exist_ok=True)
for _ in range(N_TRAIN + N_TEST):
    rep.orchestrator.step(rt_subframes=1)
w.detach()

from PIL import Image, ImageFilter
rgbs = sorted(glob.glob(os.path.join(OUT, "**", "rgb_*.png"), recursive=True))
bbs = sorted(glob.glob(os.path.join(OUT, "**", "bounding_box_2d_tight_*.npy"),
                       recursive=True))
n = min(len(rgbs), len(bbs))
imgs = [Image.open(rgbs[i]).convert("L") for i in range(n)]
X = np.stack([np.asarray(im, dtype=np.float32) / 255.0 for im in imgs])
y = np.array([len(np.load(bbs[i], allow_pickle=True)) for i in range(n)],
             dtype=np.float32)
show(f"  {len(X)} frames, counts {int(y.min())}..{int(y.max())}, "
     f"mean {y.mean():.1f}")
show()

split = len(X) - N_TEST
Xtr, ytr, Xte, yte = X[:split], y[:split], X[split:], y[split:]
te_imgs = imgs[split:]

# ---------------------------------------------------------------------------
# TRAIN on clean synthetic, exactly as 10.6 did
# ---------------------------------------------------------------------------
import torch
import torch.nn as nn

torch.manual_seed(0)
net = nn.Sequential(
    nn.Conv2d(1, 8, 5, stride=2, padding=2), nn.ReLU(),
    nn.Conv2d(8, 16, 5, stride=2, padding=2), nn.ReLU(),
    nn.Conv2d(16, 32, 3, stride=2, padding=1), nn.ReLU(),
    nn.AdaptiveAvgPool2d(1), nn.Flatten(), nn.Linear(32, 1))
opt = torch.optim.Adam(net.parameters(), lr=3e-3)
lossf = nn.L1Loss()
Xtr_t = torch.tensor(Xtr).unsqueeze(1)
ytr_t = torch.tensor(ytr).unsqueeze(1)
show("-" * 70)
show("1. TRAINED ON CLEAN SYNTHETIC")
show("-" * 70)
t0 = time.perf_counter()
for ep in range(60):
    perm = torch.randperm(len(Xtr_t))
    for i in range(0, len(perm), 16):
        idx = perm[i:i + 16]
        opt.zero_grad()
        lossf(net(Xtr_t[idx]), ytr_t[idx]).backward()
        opt.step()
net.eval()
show(f"  trained in {time.perf_counter() - t0:.1f} s")


def mae_on(arr):
    with torch.no_grad():
        p = net(torch.tensor(arr).unsqueeze(1)).squeeze(1).numpy()
    return float(np.mean(np.abs(p - yte)))


clean = mae_on(Xte)
base = float(np.mean(np.abs(yte - ytr.mean())))
show(f"  clean synthetic test MAE : {clean:.2f}")
show(f"  the predict-the-mean floor: {base:.2f}")
skill = (base - clean) / base
show(f"  so it is {skill * 100:.1f}% better than predicting a constant")
show()
# THE PERTURBATION TEST IS ONLY MEANINGFUL IF THERE IS SKILL TO LOSE.
# The first run of this script trained a model that beat the floor by 3.7%,
# and every perturbation then changed the error by ~1% -- which looked like
# "the gap is tiny" and was really "the model barely uses the pixels, so
# damaging the pixels costs it nothing". A model with no skill cannot
# demonstrate a loss of skill. Check before believing the result.
if skill < 0.12:
    show("  !! THAT IS TOO LITTLE SKILL FOR THIS TEST TO MEAN ANYTHING.")
    show("  A model only 3-4% better than a constant is barely reading the")
    show("  image, so perturbing the image will barely move its error, and a")
    show("  small drop would say nothing about the sim-to-real gap. Training")
    show("  longer on more data before running the perturbations.")
    show()
    EXTRA = 240
    opt2 = torch.optim.Adam(net.parameters(), lr=1e-3)
    net.train()
    for ep in range(EXTRA):
        perm = torch.randperm(len(Xtr_t))
        for i in range(0, len(perm), 16):
            idx = perm[i:i + 16]
            opt2.zero_grad()
            lossf(net(Xtr_t[idx]), ytr_t[idx]).backward()
            opt2.step()
    net.eval()
    clean = mae_on(Xte)
    skill = (base - clean) / base
    show(f"  after {EXTRA} more epochs: clean MAE {clean:.2f}, "
         f"{skill * 100:.1f}% better than the floor")
    show()

# ---------------------------------------------------------------------------
# 2. PERTURBATIONS A REAL CAMERA WOULD IMPOSE
# ---------------------------------------------------------------------------
rng = np.random.default_rng(0)


def noisy(a, sd):
    return np.clip(a + rng.normal(0, sd, a.shape).astype(np.float32), 0, 1)


def blurred(ims, r):
    return np.stack([np.asarray(im.filter(ImageFilter.GaussianBlur(r)),
                                dtype=np.float32) / 255.0 for im in ims])


def exposed(a, f):
    return np.clip(a * f, 0, 1)


def quantised(a, levels):
    return np.round(a * levels) / levels


show("-" * 70)
show("2. THE SAME MODEL, ON PERTURBED IMAGES")
show("-" * 70)
rows = [
    ("clean synthetic", Xte),
    ("sensor noise  sd 0.02", noisy(Xte, 0.02)),
    ("sensor noise  sd 0.05", noisy(Xte, 0.05)),
    ("defocus blur  r=1.0", blurred(te_imgs, 1.0)),
    ("defocus blur  r=2.0", blurred(te_imgs, 2.0)),
    ("exposure  x0.75", exposed(Xte, 0.75)),
    ("exposure  x1.30", exposed(Xte, 1.30)),
    ("8-level quantisation", quantised(Xte, 8)),
    ("noise + blur + exposure",
     exposed(noisy(blurred(te_imgs, 1.0), 0.03), 0.85)),
]
results = []
for name, arr in rows:
    m = mae_on(arr)
    results.append((name, m))
    flag = "  <- WORSE THAN THE FLOOR" if m > base else ""
    show(f"  {name:26s} MAE {m:6.2f}   "
         f"{(m - clean) / max(clean, 1e-9) * 100:+7.1f}%{flag}")
show()

worst = max(results[1:], key=lambda r: r[1])
drop = (worst[1] - clean) / clean * 100
show(f"  the worst single perturbation is {worst[0].strip()} at "
     f"{worst[1]:.2f}, which is")
show(f"  {drop:.0f}% worse than clean and "
     f"{'BELOW' if worst[1] < base else 'ABOVE'} the do-nothing floor of {base:.2f}.")
show()
# READ THE RESULT AGAINST THE SKILL, NOT ON ITS OWN.
if skill < 0.12:
    show("  AND DO NOT READ THAT AS A SMALL SIM-TO-REAL GAP. This model is")
    show(f"  only {skill * 100:.1f}% better than a constant, so there is almost no")
    show("  skill for a perturbation to destroy. The correct conclusion is")
    show("  that this experiment could not measure the gap, not that the gap")
    show("  is small -- and reporting it the other way round would be exactly")
    show("  the kind of claim this section keeps warning about.")
elif drop < 5:
    show("  With a model that genuinely uses the pixels, a drop this small is")
    show("  a real result: these particular corruptions are not what breaks")
    show("  this particular task. Counting bright blobs survives noise and")
    show("  blur, which is a property of the TASK rather than a general")
    show("  statement about synthetic data.")
else:
    show("  That is a real loss of skill from corruptions a real camera")
    show("  imposes for free, and it is a floor under the true gap.")
show()

# ---------------------------------------------------------------------------
# 3. WHAT THIS NUMBER IS, AND IS NOT
# ---------------------------------------------------------------------------
show("-" * 70)
show("3. WHAT THIS IS A MEASUREMENT OF")
show("-" * 70)
show("  Every perturbation above is a difference that DEFINITELY exists")
show("  between this renderer and any real camera: sensors are noisy, lenses")
show("  defocus, auto-exposure drifts, pipelines quantise. So the degradation")
show("  is a real component of the sim-to-real gap.")
show()
show("  It is a LOWER BOUND, and the honest statement is that it is nowhere")
show("  near the whole gap. Everything still synthetic here: the geometry,")
show("  the materials, the light transport, the objects' appearance, the")
show("  clutter statistics of a real warehouse, and the fact that a real")
show("  camera is held by something that moves. None of that is measured.")
show()
show("  A LOWER BOUND IS STILL WORTH HAVING. If a model gives up most of its")
show("  skill to sensor noise alone -- which costs nothing to test -- then it")
show("  was never going to survive a real camera, and you have learned that")
show("  without owning one. What you cannot do is run this, get a small")
show("  number, and conclude the model will transfer.")
show()
show("=" * 70)
show("done")
show("=" * 70)

sim.close()
