# PRD — Isaac Sim Demonstrator

## Project

`isaac-sim-demonstrator`

## Goal

Create a small hands-on demonstrator for learning NVIDIA Isaac Sim on a
cloud-hosted NVIDIA GPU.

The project should establish a reproducible environment and demonstrate the
basic Isaac Sim workflow before adding robotics or drone-specific complexity.

The next implementation targets are NVIDIA’s
[Isaac Sim Basic Usage Tutorial](https://docs.isaacsim.omniverse.nvidia.com/latest/introduction/quickstart_isaacsim.html#isaac-sim-app-intro-quickstart)
(Isaac Sim 6.0.1) on this cloud setup: Standalone Python headless
(EXP-05) and the GUI / Script Editor tabs via remote display (EXP-06–08).

## Objectives

1. Run NVIDIA Isaac Sim on a cloud GPU.
2. Prefer a containerized setup that can be reproduced on different GPU hosts.
3. Run Isaac Sim headless without requiring a local NVIDIA workstation.
4. Execute a simple simulation from Python.
5. Create or load a simple scene.
6. Add a simulated camera.
7. Generate camera output from the simulation.
8. Reproduce the official Basic Usage Tutorial (Standalone Python workflow)
   with the same APIs, prims, and scene outcome as the docs.
9. Render the Isaac Sim GUI on the cloud GPU and display it on a local Mac
   so the official GUI and Extensions / Script Editor workflows can be
   followed interactively.

## Official tutorial

Source of truth:

- [Isaac Sim Basic Usage Tutorial](https://docs.isaacsim.omniverse.nvidia.com/latest/introduction/quickstart_isaacsim.html#isaac-sim-app-intro-quickstart)
- Isaac Sim **6.0.1**
- Vendor script (already in the container):
  `/isaac-sim/standalone_examples/tutorials/getting_started/getting_started.py`

The tutorial teaches the same scene in three workflows (GUI, Extensions /
Script Editor, Standalone Python). Standalone Python is already in scope
as EXP-05 (headless). GUI and Extensions need the Kit window: that window
must run on the cloud GPU and be shown on the local Mac (see Phase 3).

The official intro mentions “a moving robot.” That is the
[Basic Robot Tutorial](https://docs.isaacsim.omniverse.nvidia.com/latest/introduction/quickstart_isaacsim_robot.html),
which is a follow-up after this page. Do not implement it in this phase.

## Initial Scope (complete)

These experiments established the environment. Keep them; do not replace
them with the official tutorial.

### EXP-01 — Isaac Sim Startup

- Start Isaac Sim on the cloud GPU.
- Run in headless mode.
- Verify GPU detection.
- Execute a minimal Python script.

### EXP-02 — Simple Scene

- Create or load a basic scene.
- Add a ground plane.
- Add a simple rigid object.
- Run the simulation for a defined number of steps.

### EXP-03 — Camera

- Add a simulated camera.
- Position the camera in the scene.
- Capture an RGB image.
- Store the generated image under an output directory.

### EXP-04 — Basic Motion

- Add a movable object or simple robot.
- Change its position or velocity from Python.
- Observe the result through the simulated camera.

## Phase 2 — Official Basic Usage Tutorial

### EXP-05 — Official Basic Usage (Standalone Python)

Add a headless repo script that performs the same actions as the official
Standalone Python tab / `getting_started.py`. Prefer matching the docs
exactly (prim paths, APIs, numbers) rather than inventing a simpler scene.

It is acceptable to run NVIDIA’s `getting_started.py` as a first check if
it starts headless in this container. The deliverable is still a script
under `src/` that:

- Starts `SimulationApp` headless (same pattern as EXP-01–04).
- Uses Isaac Sim 6.0 experimental Core APIs (not Isaac Lab spawners).
- Exits cleanly without `SimulationApp.close()` (`finish()`).
- Writes at least one RGB still under `output/` so the scene can be
  inspected without a GUI.

Required scene, in this order, matching the docs:

1. **New / empty stage.**
2. **Ground plane** — `GroundPlane("/World/GroundPlane", positions=[0, 0, 0])`.
3. **Distant light** — `DistantLight("/DistantLight")` with intensity `300`.
4. **Visual cube (no physics)** — yellow `PreviewSurfaceMaterial` at
   `/Materials/yellow` (`diffuseColor` `[1.0, 1.0, 0.0]`); `Cube` at
   `/visual_cube`, position `[0, 0.5, 1.0]`, size `0.3`. Do **not** apply
   `RigidPrim` or collision.
5. **Dynamic cube (physics + collision)** — cyan `PreviewSurfaceMaterial`
   at `/Materials/cyan` (`diffuseColor` `[0.0, 1.0, 1.0]`); `Cube` at
   `/dynamic_cube`, position `[0, -0.5, 1.5]`, size `0.3`; then
   `RigidPrim("/dynamic_cube")` and
   `GeomPrim("/dynamic_cube", apply_collision_apis=True)`.
6. **Move / rotate / scale** the visual cube with `XformPrim`:
   translation `[1.5, 1.2, 1.0]`, quaternion `[0.7, 0.7, 0, 1]` (wxyz),
   scale `[1, 1.5, 0.2]`.
7. **Look up properties** — print prim paths and world poses for
   `/visual_cube` and `/dynamic_cube` before play and after stepping.
8. **Play and step** the timeline for a fixed number of frames (enough
   for the dynamic cube to fall and rest on the ground).

Expected result (what the tutorial asks you to see):

- `/visual_cube` does **not** fall (no mass / collision).
- `/dynamic_cube` **does** fall under gravity and collide with the ground.
- The visual cube’s pose after the `XformPrim` call matches the
  translation / orientation / scale above.

Optional (from the Extensions tab; not required to call EXP-05 done):

- A second visual cube via raw USD (`UsdGeom.Cube` at `/visual_cube_usd`).
- Apply `RigidPrim` + `GeomPrim` to an already-spawned visual cube.

## Phase 3 — Remote GUI (cloud render, local Mac display)

The official GUI and Script Editor steps need the real Kit UI (menus,
gizmos, Property panel, Script Editor). The Mac has no NVIDIA GPU, so
Isaac Sim stays on RunPod. The Mac is only a viewer and input device.

```text
Mac (display + mouse/keyboard)
    ↑  TCP (HTTP / WebSocket), or SSH tunnel
RunPod container
    Isaac Sim Kit GUI  →  virtual X display (GPU)  →  VNC/noVNC
```

### Constraint: NVIDIA livestream will not work on RunPod

NVIDIA’s supported remote UI is WebRTC livestream
([Livestream Clients](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/manual_livestream_clients.html)):

- Native **Isaac Sim WebRTC Streaming Client** (macOS app)
- Or a Chromium **web viewer** (TCP 8210)

Both need:

- TCP `49100` (signaling)
- **UDP `47998`** (media / SRTP)
- GPU with **NVENC** (RTX 4090 is fine; A100 is not)
- For containers: `--network=host` and a reachable public IP
  (`ISAACSIM_HOST` / `primaryStream/publicIp`)

RunPod (and similar proxy GPU clouds) forward **TCP only**. Signaling
can connect; video never arrives (grey/black client). Do not spend
implementation time forcing WebRTC on RunPod.

Use NVIDIA livestream only if the host later has inbound UDP (for
example a VM with a public IP and UDP 47998 open). Document that path
as optional, not the default for this project.

### Required path on RunPod: virtual display + noVNC

Isaac Sim draws its normal GUI into a virtual X screen on the GPU.
That screen is served over **TCP HTTP**, which RunPod’s proxy can
carry.

```text
Isaac Sim GUI (DISPLAY=:1)
    → Xorg + NVIDIA dummy screen (not Xvfb)
    → x11vnc (localhost :5900)
    → websockify + noVNC (:8080)
    → Mac SSH -L 8080 (TCP; not ssh.runpod.io)
    → browser
```

Xvfb cannot host Isaac 6 RTX (`advanceCurrentFrame: backbuffers are not
initialized`). Proven 2026-08-30: Kit window, RTX 4090 overlay, ~76 FPS.

Requirements:

- RunPod template: RTX GPU, `22/tcp`, `NVIDIA_DRIVER_CAPABILITIES=all`.
- Host-injected `nvidia_drv.so` (do not apt `xserver-xorg-video-nvidia-*`).
- SSH tunnel so VNC is not on the public Internet. NVIDIA livestream
  has no auth; do not copy that model.
- Launch Kit **with a display** (`DISPLAY=:1`, not `--no-window` /
  headless `SimulationApp`). GLFW must see Xorg.
- Do not run a headless experiment (`python.sh src/exp0N_*.py`) at the
  same time as the GUI Kit process on a 24 GB GPU unless memory is
  confirmed free.
- `docker/gui.sh` starts Xorg, noVNC, and Isaac. The Dockerfile bakes
  those packages; `gui start` can apt-install them on an older image.

That **does** change `Dockerfile.runpod` / `entrypoint.runpod.sh` /
`gui.sh`, so it triggers an image rebuild. Experiment scripts stay in git.

### EXP-06 — Remote GUI session

- Start Isaac Sim’s GUI on the cloud GPU with a virtual display.
- Open that desktop from the Mac (browser noVNC, or VNC over SSH).
- Confirm the Kit window is interactive (viewport, menus).
- Confirm GPU is used (`nvidia-smi` shows `kit` while the UI is open).
- Write the exact RunPod ports, env vars, and Mac URL in the README.

Success: a person at the Mac can click through Kit without a local
NVIDIA GPU.

### EXP-07 — Official GUI workflow

Using the EXP-06 session, follow the official **GUI** tab:

1. File > New
2. Create > Physics > Ground Plane
3. Create > Lights > Distant Light
4. Create > Shape > Cube (visual only; Play does nothing to it)
5. Move / rotate / scale with gizmos (W / E / R) or the Property panel
6. Add **Rigid Body with Colliders Preset** on `/World/Cube`
7. Play: the cube falls and collides with the ground

Optional evidence: one screenshot from the streamed viewport under
`output/` (Mac screenshot is acceptable).

### EXP-08 — Official Extensions / Script Editor workflow

Using the EXP-06 session, follow the official **Extensions** tab:

1. Window > Script Editor
2. Run the documented snippets **in order** (new stage, ground, light,
   visual + test cubes, materials, physics on `/test_cube` and/or
   `/dynamic_cube`, `XformPrim` / raw USD transforms)
3. Play: visual cubes stay put; dynamic / test cubes fall
4. Note hot-reload: edit a snippet, save, re-run without shutting Kit

Same APIs as EXP-05; the point is interactive Kit, not another
standalone `python.sh` script.

## Out of Scope for This Phase

- Making NVIDIA WebRTC livestream work on RunPod (UDP is not available)
- Building a custom TURN/ICE stack to carry WebRTC over TCP
- The follow-on Basic Robot Tutorial (Franka / Nova Carter)
- PX4
- ROS / ROS 2
- Drone flight simulation
- LiDAR
- Reinforcement learning
- Isaac Lab
- Multi-GPU execution
- Jetson deployment

## Environment

Target environment:

- RunPod or comparable NVIDIA GPU cloud
- RT-capable NVIDIA GPU
- Linux
- NVIDIA Isaac Sim **6.0.1**
- Docker/container-based deployment where practical
- Python for simulation control
- Experiment scripts live in the git repo, not baked into the image
- For GUI: virtual X display + noVNC (TCP HTTP) on RunPod; Mac is the
  viewer. NVIDIA WebRTC livestream is documented only for UDP-capable
  hosts.

The project should avoid depending on manually configured state on a
particular cloud VM.

## Repository Structure

```text
isaac-sim-demonstrator/
├── docker/
├── src/
├── output/
├── PRD.md
├── README.md
└── TASKS.md
```

## Success Criteria

EXP-01–04 remain complete when:

1. Isaac Sim starts successfully on a cloud GPU.
2. A simulation can be executed headless.
3. A simple scene can be created from Python.
4. A camera can render an image from the scene.
5. The generated image can be retrieved and viewed locally.
6. The setup can be recreated from the repository documentation.

EXP-05 is complete when:

1. A headless script recreates the official Basic Usage scene with the
   prims and APIs listed above.
2. Logs show `/visual_cube` stays off the ground and `/dynamic_cube`
   falls onto the ground plane.
3. An RGB image of that scene is written under `output/` and can be
   copied off the host.
4. The script is documented in the project README with the official
   tutorial URL.

EXP-06–08 are complete when:

1. The Isaac Sim GUI is rendered on the cloud GPU and displayed on a
   local Mac over TCP (noVNC or SSH-tunneled VNC).
2. The official GUI tab can be completed from that Mac session
   (gizmos, Property panel, Play).
3. The official Script Editor tab can be completed from that session.
4. README documents ports, how to open the viewer on macOS, and why
   WebRTC livestream is not used on RunPod.

## Future Direction

After EXP-06–08, the next official NVIDIA page is the Basic Robot Tutorial
(`getting_started_robot.py`: Franka arm + Nova Carter). After that,
extend toward a drone simulation:

```text
Isaac Sim
    ↓
Official Basic Usage, Standalone Python (EXP-05)
    ↓
Remote GUI on Mac (EXP-06) + official GUI / Script Editor (EXP-07–08)
    ↓
Official Basic Robot Tutorial
    ↓
Simulated Camera / LiDAR
    ↓
Drone Model
    ↓
PX4 Integration
    ↓
Autonomous Perception and Control Experiments
```
