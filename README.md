# Isaac Sim Demonstrator

Small headless NVIDIA Isaac Sim project for a cloud GPU. It starts Isaac Sim
from Python, builds a simple scene, captures RGB from a simulated camera, and
writes images you can copy off the host.

Target: **Isaac Sim 6.0.1** (`nvcr.io/nvidia/isaac-sim:6.0.1`, including the
RunPod image that reports `6.0.1-rc.7`).

APIs used: experimental Core (`Cube`, `RigidPrim`, `GroundPlane`) and
`isaacsim.sensors.experimental.rtx` (`RtxCamera` + `CameraSensor`).

The RunPod image is Isaac Sim + SSH + a virtual desktop for the GUI.
Clone this repo onto the pod to get `src/`. Image rebuilds are documented
in [docker/README.md](docker/README.md).

## Experiments

| Script | PRD item | What it does |
| --- | --- | --- |
| `src/exp01_startup.py` | EXP-01 | Headless Kit start, GPU check, a few app updates |
| `src/exp02_simple_scene.py` | EXP-02 | Ground plane + rigid cube, 120 physics steps |
| `src/exp03_camera.py` | EXP-03 | Camera looking at the cube, PNG under `output/` |
| `src/exp04_motion.py` | EXP-04 | Apply cube velocity, save a short RGB sequence |
| `src/exp05_basic_usage.py` | EXP-05 | NVIDIA `getting_started.py` with `headless=True` and an exit after the loop |
| `docker/gui.sh` | EXP-06 | Isaac Sim GUI on the cloud GPU, viewed on a Mac via noVNC (TCP) |
| *(Kit GUI)* | EXP-07 | Official GUI tab: menus, gizmos, Play |
| `src/exp08/*.py` | EXP-08 | Official Script Editor snippets (paste into Kit, do not run with `python.sh`) |

Out of scope for this version: PX4, ROS 2, drones, LiDAR, Isaac Lab, RL,
NVIDIA WebRTC livestream on RunPod (UDP is not forwarded).

## Run

From an Isaac Sim 6.0.1 container (`/isaac-sim/python.sh` exists), at the
repo root:

```bash
chmod +x docker/run.sh
./docker/run.sh exp01
./docker/run.sh exp02
./docker/run.sh exp03
./docker/run.sh exp04
./docker/run.sh exp05
```

Headless experiments each start their **own** Kit process. Do not run them
while the GUI Kit from EXP-06 is using the GPU.

Or all five headless experiments, in order:

```bash
./docker/run.sh all
```

Equivalent direct calls:

```bash
/isaac-sim/python.sh "$PWD/src/exp01_startup.py"
/isaac-sim/python.sh "$PWD/src/exp03_camera.py"
```

`docker/run.sh` uses `/isaac-sim/python.sh` automatically when that file exists.

The first Kit launch compiles shaders and can take several minutes.

## Remote GUI (EXP-06)

Isaac Sim renders on the RunPod GPU. The Mac only shows the window.

NVIDIA’s WebRTC streaming client needs **UDP 47998**. RunPod does not
forward UDP, so that client stays grey. Use noVNC over TCP instead.

Rebuild the RunPod image after this change (Xvfb / noVNC packages). On
the template, expose **HTTP or TCP 8080**. Optional env:

- `VNC_PASSWORD` — VNC password (generated on first `start` if unset)
- `ISAAC_GUI=1` — start the virtual desktop at pod boot (not Isaac Sim)

On the pod:

```bash
./docker/run.sh gui start
# or: isaac-gui.sh start

./docker/run.sh gui smoke    # xclock; confirm the Mac browser first
./docker/run.sh gui isaac    # Isaac Sim GUI; first load is slow
./docker/run.sh gui status
```

On the Mac, SSH tunnel (recommended):

```bash
ssh -L 8080:127.0.0.1:8080 <runpod-ssh> -i ~/.ssh/id_ed25519
open http://127.0.0.1:8080/vnc.html?autoconnect=1
```

Enter the VNC password from `gui status`. You should get an interactive
Kit window. `nvidia-smi` on the pod should show `kit`.

If you exposed port 8080 on the RunPod proxy instead:

```text
https://<POD_ID>-8080.proxy.runpod.net/vnc.html?autoconnect=1
```

Quit with **File > Exit** in the streamed app, then `./docker/run.sh gui stop`.
To also kill Kit: `./docker/run.sh gui stop --isaac`.

## Official GUI tab (EXP-07)

With the EXP-06 session open, follow the
[Basic Usage Tutorial GUI tab](https://docs.isaacsim.omniverse.nvidia.com/latest/introduction/quickstart_isaacsim.html):

1. File > New
2. Create > Physics > Ground Plane
3. Create > Lights > Distant Light
4. Create > Shape > Cube (Play: the cube should not fall)
5. W / E / R gizmos, or type values in the Property panel
6. Select `/World/Cube` → Property panel → Add → Physics →
   **Rigid Body with Colliders Preset**
7. Play: the cube falls onto the ground

Optional: save a Mac screenshot under `output/`.

## Official Script Editor tab (EXP-08)

Still in the EXP-06 session: **Window > Script Editor**. Open a new tab
for each file under `src/exp08/`, paste, Run, in this order:

| File | What it does |
| --- | --- |
| `01_ground_plane.py` | New stage + ground |
| `02_distant_light.py` | Distant light intensity 300 |
| `03_visual_cubes.py` | Yellow visual cube + cyan test cube |
| `04_usd_cube.py` | Optional raw USD cube |
| `05_dynamic_cube.py` | Red cube with rigid body + collision |
| `06_physics_on_test_cube.py` | Physics on `/test_cube` |
| `07_xform.py` | Move / rotate / scale `/test_cube` |

Play: visual cubes stay put; cyan/red cubes fall. Edit a snippet, save,
re-run — that is hot-reload. Do **not** run these files with
`/isaac-sim/python.sh`.

## Retrieve images

After EXP-03 / EXP-04 / EXP-05:

```text
output/exp03_rgb.png
output/exp04_frame_01.png
...
```

Copy them to your laptop with `scp` or the RunPod file browser.

## Layout

```text
isaac-sim-demonstrator/
├── docker/                 # Run helper, gui.sh; image notes in docker/README.md
├── src/                    # Headless experiments + src/exp08 Script Editor snippets
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
| GLFW initialization failed / no Kit window | GUI needs `docker/gui.sh start` so `DISPLAY` exists. Headless `python.sh` scripts are supposed to run without a window. |
| WebRTC client grey/black screen | Expected on RunPod. Use noVNC (`gui start`), not the NVIDIA streaming client. |
| noVNC asks for a password | `./docker/run.sh gui status` prints it (or `VNC_PASSWORD`). |
| `Missing Xvfb` | Current pod image is older than this change. Wait for GH Actions rebuild, pull the new tag. |
| Two Kit processes / SSH drop | Stop GUI or headless before starting the other. `gui stop --isaac` then `nvidia-smi`. |

## Success criteria

You should be able to:

1. Start Isaac Sim on the cloud GPU (EXP-01).
2. Run a headless simulation (EXP-01 / EXP-02).
3. Create a scene from Python (EXP-02).
4. Render an RGB image from a simulated camera (EXP-03).
5. Copy that image off the host and open it locally.
6. Recreate the setup from this README on another GPU machine.
7. Run the official Basic Usage standalone example headless (EXP-05) until
   `python.sh` returns.
8. Open the Isaac Sim GUI on a Mac via noVNC (EXP-06).
9. Complete the official GUI tab (EXP-07) and Script Editor tab (EXP-08).
