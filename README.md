# Isaac Sim Demonstrator

Small headless NVIDIA Isaac Sim project for a cloud GPU. It starts Isaac Sim
from Python, builds a simple scene, captures RGB from a simulated camera, and
writes images you can copy off the host.

Target: **Isaac Sim 6.0.1** (`nvcr.io/nvidia/isaac-sim:6.0.1`, including the
RunPod image that reports `6.0.1-rc.7`).

APIs used: experimental Core (`Cube`, `RigidPrim`, `GroundPlane`) and
`isaacsim.sensors.experimental.rtx` (`RtxCamera` + `CameraSensor`).

The RunPod image is Isaac Sim + SSH + Xorg/noVNC for the GUI. Clone or
rsync this repo onto the pod to get `src/` and `docker/gui.sh`. Image
rebuilds are documented in [docker/README.md](docker/README.md).

Python lives under `src/<origin>/`. Do not mix origins in one folder.

| Origin | README |
| --- | --- |
| Environment Check | [src/environment_check/README.md](src/environment_check/README.md) |
| Isaac Tutorial | [src/isaac_tutorial/README.md](src/isaac_tutorial/README.md) |
| Udemy: NVIDIA Isaac Sim from Scratch | [src/udemy_nvidia_isaacsim_from_scratch/README.md](src/udemy_nvidia_isaacsim_from_scratch/README.md) |

Out of scope for this version: PX4, ROS 2, drones, LiDAR, Isaac Lab, RL,
NVIDIA WebRTC livestream on RunPod (UDP is not forwarded). The Udemy
origin has its own ROS 2 / Replicator / RL scripts; those are course
assets, not this repo’s Phase 1 scope.

## Run

From an Isaac Sim 6.0.1 container (`/isaac-sim/python.sh` exists), at the
repo root. Script lists and commands are in each origin README.

```bash
chmod +x docker/run.sh
./docker/run.sh all
```

`all` runs Environment Check `01`–`04` then Isaac Tutorial
`01_basic_usage`. Headless experiments each start their **own** Kit
process. Do not run them while the GUI Kit from EXP-06 is using the GPU.

`docker/run.sh` uses `/isaac-sim/python.sh` automatically when that file
exists. The first Kit launch compiles shaders and can take several
minutes.

## Remote GUI (EXP-06)

Isaac Sim renders on the RunPod GPU. The Mac only shows the window.

```text
Isaac (DISPLAY=:1) → Xorg + NVIDIA dummy screen → x11vnc :5900 → noVNC :8080 → Mac SSH -L
```

NVIDIA’s WebRTC client needs **UDP 47998**. RunPod does not forward UDP,
so that client stays black. **Xvfb cannot host Isaac RTX** (`backbuffers
are not initialized`). Use Xorg + NVIDIA + noVNC over **TCP SSH**.

### Recreate a pod

1. Deploy the RunPod image (Docker Hub tag after CI rebuild, or the
   current image and let `gui start` apt-install Xorg/noVNC).
2. Template: RTX 4090 (or other RTX with NVENC). Expose **22/tcp**.
   Optional: `NVIDIA_DRIVER_CAPABILITIES=all`, `PUBLIC_KEY` (your
   `~/.ssh/id_ed25519.pub` so TCP SSH works at boot), `VNC_PASSWORD`,
   `ISAAC_GUI=1` (desktop at boot, not Isaac).
   `gui start` apt-installs Xorg/noVNC if needed and extracts
   `nvidia_drv.so` to match the host driver if it was not injected.
3. Rsync this repo (use Connect → **SSH over exposed TCP**, not
   `ssh.runpod.io`):

```bash
rsync -av --delete --no-owner --no-group --exclude .git --exclude .venv --exclude __pycache__ \
  -e "ssh -i ~/.ssh/id_ed25519 -p <TCP_SSH_PORT>" \
  ./ root@<PUBLIC_IP>:/workspace/
```

4. On the pod:

```bash
cd /workspace
./docker/run.sh gui probe    # GPU, nvidia_drv.so, /dev/dri
./docker/run.sh gui start    # installs packages if the image is old
./docker/run.sh gui smoke    # xclock on the desktop
./docker/run.sh gui isaac    # Kit; first load is slow
./docker/run.sh gui status   # VNC password
```

5. On the Mac (keep this tunnel open):

```bash
ssh -L 8080:127.0.0.1:8080 root@<PUBLIC_IP> -p <TCP_SSH_PORT> -i ~/.ssh/id_ed25519
open http://127.0.0.1:8080/vnc.html?autoconnect=1
```

Enter the password from `gui status`. You want the Openbox desktop, then
a Kit window (viewport overlay names the RTX GPU). `nvidia-smi` shows
`kit`.

Proxy SSH (`*@ssh.runpod.io`) cannot `-L`. Do not publish 8080 unless you
set `VNC_PASSWORD`.

Quit with **File > Exit**, then `./docker/run.sh gui stop`.
To also kill Kit: `./docker/run.sh gui stop --isaac`.

GUI and Script Editor tabs for NVIDIA Basic Usage:
[src/isaac_tutorial/README.md](src/isaac_tutorial/README.md).

## Layout

```text
isaac-sim-demonstrator/
├── docker/                 # Run helper, gui.sh; image notes in docker/README.md
├── src/
│   ├── environment_check/                  # GPU smoke tests + README
│   ├── isaac_tutorial/                     # NVIDIA Basic Usage + README
│   └── udemy_nvidia_isaacsim_from_scratch/ # Udemy course assets + README
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
| GLFW initialization failed / no Kit window | Run `gui start` first so Xorg is on `DISPLAY=:1`. Headless `python.sh` does not need a window. |
| WebRTC client grey/black screen | Expected on RunPod (no UDP). Use noVNC, not the NVIDIA streaming client. |
| `backbuffers are not initialized` | Xvfb is running, or `gui start` is an old script. Stop it and use this repo’s Xorg `gui.sh`. |
| `glxinfo` shows llvmpipe | NVIDIA X driver not presenting. `gui probe`; do not apt-install `xserver-xorg-video-nvidia-*`. |
| SSH `-L` “unsupported channel” | You used `ssh.runpod.io`. Use the public IP and TCP port from Connect. |
| noVNC Connection refused | `gui start` is not up, or the tunnel is to the wrong host. `gui status`. |
| noVNC asks for a password | `./docker/run.sh gui status` prints it (or `VNC_PASSWORD`). |
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
