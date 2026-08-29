# PRD — Isaac Sim Demonstrator

## Project

`isaac-sim-demonstrator`

## Goal

Create a small hands-on demonstrator for learning NVIDIA Isaac Sim on a
cloud-hosted NVIDIA GPU.

The project should establish a reproducible environment and demonstrate the
basic Isaac Sim workflow before adding robotics or drone-specific complexity.

## Objectives

1. Run NVIDIA Isaac Sim on a cloud GPU.
2. Prefer a containerized setup that can be reproduced on different GPU hosts.
3. Run Isaac Sim headless without requiring a local NVIDIA workstation.
4. Execute a simple simulation from Python.
5. Create or load a simple scene.
6. Add a simulated camera.
7. Generate camera output from the simulation.

## Initial Scope

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

## Out of Scope for Initial Version

The first version will deliberately not include:

- PX4
- ROS / ROS 2
- Drone flight simulation
- LiDAR
- Reinforcement learning
- Isaac Lab
- Multi-GPU execution
- Jetson deployment

These can be added as follow-up experiments once the basic Isaac Sim
environment is understood.

## Environment

Target environment:

- RunPod or comparable NVIDIA GPU cloud
- RT-capable NVIDIA GPU
- Linux
- NVIDIA Isaac Sim
- Docker/container-based deployment where practical
- Python for simulation control

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

The initial demonstrator is complete when:

1. Isaac Sim starts successfully on a cloud GPU.
2. A simulation can be executed headless.
3. A simple scene can be created from Python.
4. A camera can render an image from the scene.
5. The generated image can be retrieved and viewed locally.
6. The setup can be recreated from the repository documentation.

## Future Direction

After completing the initial experiments, extend the project toward a drone
simulation:

```text
Isaac Sim
    ↓
Simulated Camera / LiDAR
    ↓
Drone Model
    ↓
PX4 Integration
    ↓
Autonomous Perception and Control Experiments
```