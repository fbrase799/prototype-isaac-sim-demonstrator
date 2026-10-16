# 10.6: training a small detector on the generated data.
#
# The curriculum note on this lecture is "real numbers or none", and that is the
# whole design constraint. It would be easy to describe training, show a loss
# curve from somewhere else, and assert that synthetic data works. This trains a
# real model on the real dataset generated in 10.5 and reports what it actually
# achieves, including if that is bad.
#
# THE TASK IS DELIBERATELY MODEST. A full object detector needs hours and a GPU
# budget this course does not have (2.6 established the hardware floor). So the
# task is the honest small version: given a 640x480 warehouse frame, predict
# HOW MANY labelled objects are in it -- a counting regression on real synthetic
# images with real free labels. It is a genuine supervised task on this data, it
# trains on CPU in minutes, and every number below is measured.
#
# WHAT IS MEASURED:
#   1. The dataset the model actually sees: size, label distribution, and the
#      1-to-230 imbalance 10.5 found, because that is what makes this hard.
#   2. A BASELINE THAT IS NOT LEARNING: predict the mean count for every image.
#      Any model must beat this or it has learned nothing. Most tutorials skip
#      this and it is the difference between a result and a plot.
#   3. The trained model, on a held-out split, against that baseline.
#   4. WHERE IT FAILS, by count bucket, because a single average hides whether
#      the model works on the frames you care about.
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

OUT = "/workspace/out/s10_6"
N_TRAIN, N_TEST = 96, 32
RES = (160, 120)          # small on purpose: CPU training, and it still works


def show(l=""):
    print("R106> " + l, flush=True)


show("=" * 70)
show("10.6  TRAINING A SMALL DETECTOR -- real numbers or none")
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
for _ in range(3):                      # warm-up, discarded (10.4)
    rep.orchestrator.step(rt_subframes=1)
shutil.rmtree(OUT, ignore_errors=True)
os.makedirs(OUT, exist_ok=True)

show("-" * 70)
show(f"1. GENERATING {N_TRAIN + N_TEST} FRAMES AT {RES[0]}x{RES[1]}")
show("-" * 70)
t0 = time.perf_counter()
for _ in range(N_TRAIN + N_TEST):
    rep.orchestrator.step(rt_subframes=1)
gen = time.perf_counter() - t0
w.detach()
show(f"  {N_TRAIN + N_TEST} frames in {gen:.1f} s ({gen / (N_TRAIN + N_TEST) * 1000:.0f} ms/frame)")

# ---------------------------------------------------------------------------
# LOAD: images and their object counts
# ---------------------------------------------------------------------------
from PIL import Image
rgbs = sorted(glob.glob(os.path.join(OUT, "**", "rgb_*.png"), recursive=True))
bbs = sorted(glob.glob(os.path.join(OUT, "**", "bounding_box_2d_tight_*.npy"),
                       recursive=True))
n = min(len(rgbs), len(bbs))
X, y = [], []
for i in range(n):
    im = np.asarray(Image.open(rgbs[i]).convert("L"), dtype=np.float32) / 255.0
    X.append(im)
    y.append(len(np.load(bbs[i], allow_pickle=True)))
X = np.stack(X)
y = np.array(y, dtype=np.float32)
show(f"  loaded {len(X)} images {X.shape[1]}x{X.shape[2]}, "
     f"counts min {int(y.min())} max {int(y.max())} mean {y.mean():.1f}")
show()

# ---------------------------------------------------------------------------
# 2. THE BASELINE THAT IS NOT LEARNING
# ---------------------------------------------------------------------------
split = len(X) - N_TEST
Xtr, ytr, Xte, yte = X[:split], y[:split], X[split:], y[split:]
show("-" * 70)
show("2. THE BASELINE ANY MODEL MUST BEAT")
show("-" * 70)
mean_pred = float(ytr.mean())
base_mae = float(np.mean(np.abs(yte - mean_pred)))
show(f"  predict {mean_pred:.1f} for every image: test MAE {base_mae:.2f}")
show("  A model that does not beat this has learned nothing, however good its")
show("  loss curve looks. This line is why the lecture exists.")
show()

# ---------------------------------------------------------------------------
# 3. TRAIN
# ---------------------------------------------------------------------------
show("-" * 70)
show("3. TRAINING")
show("-" * 70)
try:
    import torch
    import torch.nn as nn
except Exception as e:
    show(f"  torch unavailable: {type(e).__name__}: {e}")
    show("  NO MODEL, THEREFORE NO NUMBERS. Say that rather than describing a")
    show("  result that was never produced.")
    sim.close()
    raise SystemExit(0)

torch.manual_seed(0)
dev = "cpu"
Xtr_t = torch.tensor(Xtr).unsqueeze(1)
ytr_t = torch.tensor(ytr).unsqueeze(1)
Xte_t = torch.tensor(Xte).unsqueeze(1)
yte_t = torch.tensor(yte).unsqueeze(1)

net = nn.Sequential(
    nn.Conv2d(1, 8, 5, stride=2, padding=2), nn.ReLU(),
    nn.Conv2d(8, 16, 5, stride=2, padding=2), nn.ReLU(),
    nn.Conv2d(16, 32, 3, stride=2, padding=1), nn.ReLU(),
    nn.AdaptiveAvgPool2d(1), nn.Flatten(), nn.Linear(32, 1),
).to(dev)
nparam = sum(p.numel() for p in net.parameters())
show(f"  a {nparam}-parameter CNN, CPU only")

opt = torch.optim.Adam(net.parameters(), lr=3e-3)
lossf = nn.L1Loss()
EPOCHS, BS = 60, 16
t0 = time.perf_counter()
for ep in range(EPOCHS):
    perm = torch.randperm(len(Xtr_t))
    net.train()
    for i in range(0, len(perm), BS):
        idx = perm[i:i + BS]
        opt.zero_grad()
        loss = lossf(net(Xtr_t[idx]), ytr_t[idx])
        loss.backward()
        opt.step()
    if ep % 15 == 14 or ep == 0:
        net.eval()
        with torch.no_grad():
            te = float(lossf(net(Xte_t), yte_t))
        show(f"    epoch {ep + 1:3d}  train {float(loss):7.2f}  test MAE {te:7.2f}")
train_s = time.perf_counter() - t0
net.eval()
with torch.no_grad():
    pred = net(Xte_t).squeeze(1).numpy()
mae = float(np.mean(np.abs(pred - yte)))
show(f"  trained in {train_s:.1f} s")
show()

# ---------------------------------------------------------------------------
# 4. THE RESULT, AGAINST THE BASELINE, AND WHERE IT FAILS
# ---------------------------------------------------------------------------
show("-" * 70)
show("4. THE RESULT")
show("-" * 70)
show(f"  baseline (predict the mean) : MAE {base_mae:6.2f}")
show(f"  trained model               : MAE {mae:6.2f}")
imp = (base_mae - mae) / base_mae * 100
show(f"  improvement                 : {imp:6.1f}%")
show()
if mae < base_mae * 0.95:
    show("  It beat the baseline, so something was genuinely learned from")
    show("  pixels. That is the claim, and it is a modest one.")
else:
    show("  IT DID NOT BEAT THE BASELINE. Report that. A model that cannot")
    show("  beat 'predict the mean' has learned nothing, and no amount of")
    show("  loss-curve prose changes it. With 96 training images on a CPU")
    show("  that is an entirely plausible outcome, and pretending otherwise")
    show("  would be the exact failure this section keeps warning about.")
show()
show("  by count bucket, where a single average hides the truth:")
for lo, hi in ((0, 20), (20, 60), (60, 1000)):
    m = (yte >= lo) & (yte < hi)
    if m.sum():
        show(f"    {lo:3d}-{hi if hi < 1000 else '+':>4}  n={int(m.sum()):3d}  "
             f"model MAE {np.mean(np.abs(pred[m] - yte[m])):6.2f}   "
             f"baseline {np.mean(np.abs(yte[m] - mean_pred)):6.2f}")
show()
show("  THE HONEST LIMIT OF THIS LECTURE: this is a counting task on 96")
show("  training images at 160x120, not an object detector. It exists to")
show("  show the SHAPE of the exercise -- generate, split, beat a baseline,")
show("  break the result down -- on real numbers rather than borrowed ones.")
show()
show("=" * 70)
show("done")
show("=" * 70)

sim.close()
