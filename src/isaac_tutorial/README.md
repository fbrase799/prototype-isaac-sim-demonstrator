# Isaac Tutorial

NVIDIA [Basic Usage](https://docs.isaacsim.omniverse.nvidia.com/latest/introduction/quickstart_isaacsim.html),
plus the remote GUI needed to follow the GUI and Script Editor tabs.

| Script | PRD item | What it does |
| --- | --- | --- |
| `01_basic_usage.py` | EXP-05 | NVIDIA `getting_started.py` with `headless=True` and an exit after the loop |
| `docker/gui.sh` | EXP-06 | Isaac Sim GUI on the cloud GPU, viewed on a Mac via noVNC (TCP) |
| *(Kit GUI)* | EXP-07 | Official GUI tab: menus, gizmos, Play |
| `script_editor/*.py` | EXP-08 | Official Script Editor snippets (paste into Kit, do not run with `python.sh`) |

Remote GUI setup (EXP-06): [README.md](../../README.md#remote-gui-exp-06).

## Run (EXP-05)

From an Isaac Sim 6.0.1 container, at the repo root:

```bash
chmod +x docker/run.sh
./docker/run.sh 01_basic_usage
```

Equivalent:

```bash
/isaac-sim/python.sh "$PWD/src/isaac_tutorial/01_basic_usage.py"
```

Do not run this while the GUI Kit is using the GPU. Shared runner notes:
[README.md](../../README.md#run).

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
for each file under `script_editor/`, paste, Run, in this order:

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
