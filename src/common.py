"""Shared helpers for the Isaac Sim 6.0 demonstrator experiments.

Import this module only AFTER creating ``SimulationApp``. Isaac Sim loads Kit
extensions at app startup, so ``omni`` / ``isaacsim`` imports fail if they run
first.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path
from typing import Optional, Sequence, Tuple

import numpy as np

_REPO_OUTPUT = Path(__file__).resolve().parent.parent / "output"
OUTPUT_DIR = Path(os.environ.get("ISAAC_DEMO_OUTPUT", str(_REPO_OUTPUT)))


def log(message: str) -> None:
    print(f"[isaac-demo] {message}", flush=True)


def output_dir() -> Path:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    return OUTPUT_DIR


def as_numpy(value) -> np.ndarray:
    """Convert Warp / numpy / tuple-wrapped sensor buffers to ``ndarray``."""
    if value is None:
        raise ValueError("expected an array, got None")
    if isinstance(value, tuple):
        value = value[0]
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)


def log_gpu_info() -> None:
    """Verify that the process can see an NVIDIA GPU."""
    log("GPU detection")
    try:
        result = subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=index,name,driver_version,memory.total",
                "--format=csv",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        text = (result.stdout or result.stderr).strip()
        print(text)
        if result.returncode != 0:
            log("WARNING: nvidia-smi failed; Isaac Sim may not see a GPU.")
    except FileNotFoundError:
        log("WARNING: nvidia-smi is not on PATH.")


def log_app_info() -> None:
    """Print Isaac Sim / Kit identity from the container and Carb tokens."""
    version_file = Path("/isaac-sim/VERSION")
    if version_file.is_file():
        log(f"isaac-sim VERSION={version_file.read_text().strip()}")
    try:
        import carb

        tokens = carb.tokens.get_tokens_interface()
        for name in ("isaac_sim_version", "kit_version", "app_version"):
            try:
                value = tokens.resolve(f"${{{name}}}")
            except Exception:
                continue
            if value and value != f"${{{name}}}":
                log(f"{name}={value}")
    except Exception as exc:
        log(f"Could not read Kit version tokens ({exc})")


def add_basic_lights() -> None:
    """Add distant + dome lights so headless camera captures are not black."""
    import omni.usd
    from pxr import UsdLux

    stage = omni.usd.get_context().get_stage()
    distant = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    distant.CreateIntensityAttr(3000.0)
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(500.0)


def play_simulation() -> None:
    """Start the timeline so physics tensor APIs and cameras produce data."""
    import omni.timeline

    omni.timeline.get_timeline_interface().play()
    step_simulation()


def step_simulation(steps: int = 1) -> None:
    import omni.kit.app

    app = omni.kit.app.get_app()
    for _ in range(steps):
        app.update()


def build_simple_scene(cube_position: Sequence[float] = (0.0, 0.0, 0.4)):
    """Ground plane + one dynamic cube. Returns a ``RigidPrim`` for the cube."""
    import isaacsim.core.experimental.utils.stage as stage_utils
    from isaacsim.core.experimental.materials import PreviewSurfaceMaterial
    from isaacsim.core.experimental.objects import Cube, GroundPlane
    from isaacsim.core.experimental.prims import GeomPrim, RigidPrim

    stage_utils.define_prim("/World/physicsScene", "PhysicsScene")
    GroundPlane(
        "/World/groundPlane",
        sizes=10.0,
        colors=np.array([0.55, 0.55, 0.55]),
        templates=None,
        positions=[0.0, 0.0, 0.0],
    )
    add_basic_lights()

    visual = PreviewSurfaceMaterial("/World/Materials/orange")
    visual.set_input_values("diffuseColor", [0.95, 0.35, 0.15])
    cube_shape = Cube(
        paths="/World/DemoCube",
        positions=np.array([cube_position], dtype=float),
        sizes=[0.4],
        reset_xform_op_properties=True,
    )
    GeomPrim(paths=cube_shape.paths, apply_collision_apis=True)
    cube = RigidPrim(paths=cube_shape.paths, masses=[1.0])
    cube_shape.apply_visual_materials(visual)
    return cube


def look_at_wxyz(
    eye: Sequence[float],
    target: Sequence[float],
    up: Sequence[float] = (0.0, 0.0, 1.0),
) -> np.ndarray:
    """Quaternion (w, x, y, z) for a USD camera at ``eye`` looking at ``target``.

    USD cameras look along local -Z with +Y as up.
    """
    eye_v = np.asarray(eye, dtype=float)
    target_v = np.asarray(target, dtype=float)
    up_v = np.asarray(up, dtype=float)

    forward = target_v - eye_v
    norm = np.linalg.norm(forward)
    if norm < 1e-8:
        raise ValueError("eye and target are too close together")
    forward = forward / norm
    z_axis = -forward
    x_axis = np.cross(up_v, z_axis)
    x_norm = np.linalg.norm(x_axis)
    if x_norm < 1e-8:
        fallback_up = np.array([0.0, 1.0, 0.0])
        x_axis = np.cross(fallback_up, z_axis)
        x_norm = np.linalg.norm(x_axis)
    x_axis = x_axis / x_norm
    y_axis = np.cross(z_axis, x_axis)
    rotation = np.column_stack((x_axis, y_axis, z_axis))
    return _rotation_matrix_to_wxyz(rotation)


def _rotation_matrix_to_wxyz(matrix: np.ndarray) -> np.ndarray:
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * np.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * np.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    quat = np.array([w, x, y, z], dtype=float)
    return quat / np.linalg.norm(quat)


def add_rgb_camera(
    position: Sequence[float] = (3.0, 2.2, 1.6),
    target: Sequence[float] = (0.0, 0.0, 0.3),
    resolution: Tuple[int, int] = (1280, 720),
    frequency: float = 30.0,
):
    """Create an RTX camera + RGB ``CameraSensor``.

    ``resolution`` is ``(width, height)``. Isaac Sim 6.0's CameraSensor takes
    ``(height, width)``.
    """
    from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera

    width, height = resolution
    cam = RtxCamera(
        "/World/DemoCamera",
        tick_rate=float(frequency),
        positions=np.array([position], dtype=float),
        orientations=np.array([look_at_wxyz(position, target)], dtype=float),
    )
    return CameraSensor(
        cam,
        resolution=(height, width),
        annotators=["rgb"],
    )


def rgba_to_uint8_rgb(buffer) -> np.ndarray:
    array = as_numpy(buffer)
    if array.ndim == 4:
        array = array[0]
    if array.ndim != 3 or array.shape[-1] < 3:
        raise RuntimeError(f"Unexpected camera buffer shape: {array.shape}")
    rgb = array[:, :, :3]
    if np.issubdtype(rgb.dtype, np.floating):
        max_value = float(np.nanmax(rgb)) if rgb.size else 0.0
        if max_value <= 1.5:
            rgb = np.clip(rgb, 0.0, 1.0) * 255.0
        else:
            rgb = np.clip(rgb, 0.0, 255.0)
    return rgb.astype(np.uint8)


def read_rgb(sensor) -> Optional[np.ndarray]:
    try:
        result = sensor.get_data("rgb")
    except Exception:
        return None
    if result is None:
        return None
    data = result[0] if isinstance(result, tuple) else result
    if data is None:
        return None
    try:
        array = as_numpy(data)
    except Exception:
        return None
    if array.size < 16:
        return None
    return rgba_to_uint8_rgb(array)


def wait_for_rgb(sensor, max_warmup_steps: int = 40) -> np.ndarray:
    """Step until the camera produces a real image (renderer warmup)."""
    last_shape: Optional[tuple] = None
    for step_index in range(max_warmup_steps):
        step_simulation()
        rgb = read_rgb(sensor)
        if rgb is None:
            continue
        last_shape = tuple(rgb.shape)
        if min(rgb.shape[:2]) > 1:
            log(f"Camera frame ready after {step_index + 1} step(s), shape={last_shape}")
            return rgb
    raise RuntimeError(
        f"Camera did not produce RGB data after {max_warmup_steps} steps "
        f"(last shape={last_shape}). The GPU renderer may not be active."
    )


def save_rgb(image: np.ndarray, path: Path) -> Path:
    from PIL import Image

    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(image).save(path)
    log(f"Wrote {path}")
    return path


def cube_state(cube) -> Tuple[np.ndarray, np.ndarray]:
    positions, _orientations = cube.get_world_poses()
    linear_velocities, _angular = cube.get_velocities()
    return as_numpy(positions)[0], as_numpy(linear_velocities)[0]


def step_and_report(cube, steps: int, report_every: int = 30) -> None:
    from isaacsim.core.simulation_manager import SimulationManager

    for index in range(1, steps + 1):
        step_simulation()
        if index != 1 and index != steps and index % report_every != 0:
            continue
        if not SimulationManager.is_simulating():
            log(f"step={index:04d} (physics not simulating yet)")
            continue
        position, velocity = cube_state(cube)
        log(
            f"step={index:04d} pos={np.array2string(position, precision=3)} "
            f"vel={np.array2string(velocity, precision=3)}"
        )
