# Environment Check

This repo’s GPU smoke tests. They prove the cloud Isaac Sim stack before
any official tutorial.

| Script | PRD item | What it does |
| --- | --- | --- |
| `01_startup.py` | EXP-01 | Headless Kit start, GPU check, a few app updates |
| `02_simple_scene.py` | EXP-02 | Ground plane + rigid cube, 120 physics steps |
| `03_camera.py` | EXP-03 | Camera looking at the cube, PNG under `output/` |
| `04_motion.py` | EXP-04 | Apply cube velocity, save a short RGB sequence |

`common.py` is shared glue for these scripts (GPU log, output dir,
`finish()`). Import it only after `SimulationApp` is created.

## Run

From an Isaac Sim 6.0.1 container (`/isaac-sim/python.sh` exists), at the
repo root:

```bash
chmod +x docker/run.sh
./docker/run.sh 01_startup
./docker/run.sh 02_simple_scene
./docker/run.sh 03_camera
./docker/run.sh 04_motion
```

Equivalent direct calls:

```bash
/isaac-sim/python.sh "$PWD/src/environment_check/01_startup.py"
/isaac-sim/python.sh "$PWD/src/environment_check/03_camera.py"
```

Each script starts its **own** Kit process. Do not run them while the GUI
Kit is using the GPU. First launch compiles shaders and can take several
minutes.

Shared runner notes: [README.md](../../README.md#run).

## Retrieve images

After `03_camera` / `04_motion`:

```text
output/03_rgb.png
output/04_frame_01.png
...
```

Copy them to your laptop with `scp` or the RunPod file browser.
