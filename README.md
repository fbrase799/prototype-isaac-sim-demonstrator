# Isaac Sim Demonstrator

Small headless NVIDIA Isaac Sim project for a cloud GPU. It starts Isaac Sim
from Python, builds a simple scene, captures RGB from a simulated camera, and
writes images you can copy off the host.

Target: **Isaac Sim 6.0.1** (`nvcr.io/nvidia/isaac-sim:6.0.1`, including the
RunPod image that reports `6.0.1-rc.7`).

APIs used: experimental Core (`Cube`, `RigidPrim`, `GroundPlane`) and
`isaacsim.sensors.experimental.rtx` (`RtxCamera` + `CameraSensor`).

The RunPod image is Isaac Sim + SSH; clone this repo onto the pod to get
`src/`. Image rebuilds are documented in [docker/README.md](docker/README.md).

## Experiments

| Script | PRD item | What it does |
| --- | --- | --- |
| `src/exp01_startup.py` | EXP-01 | Headless Kit start, GPU check, a few app updates |
| `src/exp02_simple_scene.py` | EXP-02 | Ground plane + rigid cube, 120 physics steps |
| `src/exp03_camera.py` | EXP-03 | Camera looking at the cube, PNG under `output/` |
| `src/exp04_motion.py` | EXP-04 | Apply cube velocity, save a short RGB sequence |

Out of scope for this version: PX4, ROS 2, drones, LiDAR, Isaac Lab, RL.

## Run

From an Isaac Sim 6.0.1 container (`/isaac-sim/python.sh` exists), at the
repo root:

```bash
chmod +x docker/run.sh
./docker/run.sh exp01
./docker/run.sh exp02
./docker/run.sh exp03
./docker/run.sh exp04
```

Or all four, in order:

```bash
./docker/run.sh all
```

Equivalent direct calls:

```bash
/isaac-sim/python.sh "$PWD/src/exp01_startup.py"
/isaac-sim/python.sh "$PWD/src/exp03_camera.py"
```

`docker/run.sh` uses `/isaac-sim/python.sh` automatically when that file exists.
Each experiment starts its **own** headless Kit process. If the pod already has
`/isaac-sim/kit/kit` running for livestream/GUI, that is fine on a 24 GB GPU;
stop it first if you hit out-of-memory errors.

The first Kit launch compiles shaders and can take several minutes.

## Retrieve images

After EXP-03 / EXP-04:

```text
output/exp03_rgb.png
output/exp04_frame_01.png
...
```

Copy them to your laptop with `scp` or the RunPod file browser. No livestream
client is required. WebRTC usually does not work on RunPod (TCP-only ports).

## Layout

```text
isaac-sim-demonstrator/
├── docker/                 # Run helper; image build notes in docker/README.md
├── src/                    # Standalone Python experiments
├── output/                 # Generated RGB images (gitignored)
├── PRD.md
├── README.md
└── TASKS.md
```

## Troubleshooting

| Symptom | What to try |
| --- | --- |
| Import error for `isaacsim.sensors.camera` | This repo uses the 6.0 RTX API, not the 5.x `Camera` class |
| `nvidia-smi` empty | GPU not passed into the container |
| Black or empty camera images | Wait for warmup (scripts already step several frames); confirm RTX GPU |
| First start takes 5–15 minutes | Expected shader compile |
| `Destroying busy TaskGroup` / core dump after complete | Kit shutdown bug; ignore if you already saw `EXP-0N complete`. Scripts now exit without `close()`. |
| SSH `closed by remote host` during Kit start | Do not launch a second `python.sh` after a crash. Reconnect, run `nvidia-smi`, kill leftover `kit` processes, then start one experiment. A core dump can fill the pod disk. |
| Physics pose prints as not simulating | Timeline must be playing; `play_simulation()` in the scripts does this |

## Success criteria

You should be able to:

1. Start Isaac Sim on the cloud GPU (EXP-01).
2. Run a headless simulation (EXP-01 / EXP-02).
3. Create a scene from Python (EXP-02).
4. Render an RGB image from a simulated camera (EXP-03).
5. Copy that image off the host and open it locally.
6. Recreate the setup from this README on another GPU machine.
