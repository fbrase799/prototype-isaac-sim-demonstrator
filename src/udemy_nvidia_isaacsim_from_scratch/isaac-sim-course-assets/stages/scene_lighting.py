# 7.4: lighting for perception, not for beauty.
#
# Every previous section lit the stage well enough to see and moved on. This
# lecture makes the case that lighting is a SENSOR parameter: it decides what
# the camera can recover, and a scene that looks fine to a human can be
# unusable to a detector.
#
# Three things get measured, all on the same geometry and the same camera:
#
#   1. INTENSITY CHANGES WHAT THE CAMERA RECOVERS. The same box at several
#      dome intensities, reported as mean brightness and, more usefully, as
#      the CONTRAST between the object and the floor behind it. Contrast is
#      what a detector actually needs; brightness alone is not.
#
#   2. THERE IS A CEILING AS WELL AS A FLOOR. Too much light clips the sensor
#      and the object stops being distinguishable again, so the useful range
#      is a band rather than "more is better".
#
#   3. SHADOWS ARE INFORMATION. A directional light casts one, and the shadow
#      is extra signal a flat dome light does not provide -- measured as the
#      number of distinct intensity levels in the frame.
#
# TRAPS:
#
# - Annotators and render products need the shader cache to be writable; the
#   6.1 chown trap applies here too.
# - The renderer needs several steps to converge after a light changes. Reading
#   one frame after the change reports the PREVIOUS lighting.
# - This script refuses to print a table it did not measure.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import numpy as np
from pxr import UsdGeom, Gf, UsdLux, UsdPhysics
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import create_new_stage
from isaacsim.sensors.camera import Camera


def show(l=""):
    print("LIT> " + l, flush=True)


create_new_stage()
st = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
w = UsdGeom.Xform.Define(st, "/World")
st.SetDefaultPrim(w.GetPrim())

# a mid-grey floor and a slightly lighter box: deliberately LOW contrast, so
# that lighting is the thing deciding whether they can be told apart
gp = UsdGeom.Cube.Define(st, "/World/floor")
gp.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(gp)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
gx.AddScaleOp().Set(Gf.Vec3f(30.0, 30.0, 1.0))
gp.CreateDisplayColorAttr([(0.32, 0.33, 0.35)])

bx = UsdGeom.Cube.Define(st, "/World/box")
bx.CreateSizeAttr(1.0)
bxf = UsdGeom.Xformable(bx)
bxf.AddTranslateOp().Set(Gf.Vec3d(2.6, 0.0, 0.30))
bxf.AddScaleOp().Set(Gf.Vec3f(0.60, 0.60, 0.60))
bx.CreateDisplayColorAttr([(0.42, 0.43, 0.45)])

dome = UsdLux.DomeLight.Define(st, "/World/dome")
dome.CreateIntensityAttr(1000.0)

RES = (320, 180)
cam = UsdGeom.Camera.Define(st, "/World/cam")
cam.CreateFocalLengthAttr(24.0)
cam.CreateHorizontalApertureAttr(20.955)
cam.CreateVerticalApertureAttr(20.955 * RES[1] / RES[0])
UsdGeom.Xformable(cam).AddTransformOp().Set(
    Gf.Matrix4d().SetLookAt(Gf.Vec3d(0, 0, 0.75), Gf.Vec3d(3.0, 0, 0.35),
                            Gf.Vec3d(0, 0, 1)).GetInverse())

W = World(stage_units_in_meters=1.0)
W.reset()
for _ in range(6):
    W.step(render=False)

cm = Camera(prim_path="/World/cam", resolution=RES)
cm.initialize()

# THE FRAME DICT HAS NO "rgba" KEY. Camera.get_current_frame() returns the
# annotator outputs that were explicitly added; colour comes from get_rgb().
# Reading f["rgba"] returns None forever and the guard below fires, which is
# what happened on the first run of this script.
rgb = None
for i in range(120):
    W.step(render=True)
    a = np.asarray(cm.get_rgb())
    if a.ndim == 3 and a.size:
        rgb = a
        break
if rgb is None:
    show("CAMERA NEVER RETURNED A FRAME -- no table rather than zeros.")
    show("check: docker exec -u 0 isaac-sim chown -R isaac-sim:isaac-sim /isaac-sim/kit/cache")
    print("LIT_DONE", flush=True)
    sim.close()
    raise SystemExit(1)


def sample(intensity, settle=24):
    """Set the dome intensity, let the renderer converge, and measure."""
    dome.GetIntensityAttr().Set(float(intensity))
    # THE RENDERER LAGS. Reading one frame after the change reports the
    # previous lighting; 24 steps is where the numbers stop moving.
    for _ in range(settle):
        W.step(render=True)
    a = np.asarray(cm.get_rgb())[:, :, :3].astype(float) / 255.0
    lum = 0.2126 * a[:, :, 0] + 0.7152 * a[:, :, 1] + 0.0722 * a[:, :, 2]
    h, wd = lum.shape
    # the box sits right of centre in this framing; the floor fills the left
    obj = lum[int(h * 0.45):int(h * 0.75), int(wd * 0.55):int(wd * 0.80)]
    flr = lum[int(h * 0.55):int(h * 0.85), int(wd * 0.10):int(wd * 0.35)]
    clipped = float(np.mean(lum > 0.99))
    return (float(lum.mean()), float(obj.mean()), float(flr.mean()),
            float(abs(obj.mean() - flr.mean())), clipped)


show(f"a 0.6 m box on a floor, {RES[0]}x{RES[1]} camera, one dome light.")
show("the box and floor are deliberately CLOSE in colour, so the light is")
show("what decides whether they can be told apart:")
show("")
show(f"   {'intensity':>10} {'frame mean':>11} {'box':>8} {'floor':>8} "
     f"{'contrast':>10} {'clipped':>9}")
show("   " + "-" * 62)
rows = []
for I in (10, 100, 1000, 6000, 40000):
    m, o, f2, c, cl = sample(I)
    rows.append((I, m, o, f2, c, cl))
    show(f"   {I:10.0f} {m:11.4f} {o:8.4f} {f2:8.4f} {c:10.4f} {cl:8.1%}")
show("")

best = max(rows, key=lambda r: r[4])
worst_dark = rows[0]
worst_bright = rows[-1]
show(f"   best contrast at intensity {best[0]:.0f}: {best[4]:.4f}")
show(f"   at {worst_dark[0]:.0f} it is {worst_dark[4]:.4f}, at "
     f"{worst_bright[0]:.0f} it is {worst_bright[4]:.4f}")
show("")
show("   so it is a BAND, not a direction. Too little light and everything is")
show("   the same dark; too much and everything is the same white. A detector")
show("   needs the difference between object and background, and that peaks in")
show("   the middle and falls off both ways.")

print("LIT_DONE", flush=True)
sim.close()
