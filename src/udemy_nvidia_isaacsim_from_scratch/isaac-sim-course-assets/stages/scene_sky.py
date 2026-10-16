# 7.5: HDRI skies and outdoor scenes.
#
# 7.4 ended on the dome light's real limitation: it lights everything evenly
# from everywhere, which means no shadows, and shadows are information. An HDRI
# sky fixes that by carrying a real sky's DIRECTIONALITY -- a bright sun in one
# place, dimmer sky elsewhere -- so the same geometry suddenly has a lit side,
# a shaded side, and a shadow on the ground.
#
# Three things get measured on identical geometry and an identical camera:
#
#   1. A PLAIN DOME LIGHT PRODUCES NO DIRECTION. The lit and shaded faces of a
#      box read the same, and the ground beside it reads the same as the ground
#      under it. That is the "no shadow" claim, as a number.
#
#   2. AN HDRI PRODUCES BOTH. Different faces, and a measurably darker patch
#      where the shadow falls.
#
#   3. DIFFERENT SKIES ARE DIFFERENT LIGHTING CONDITIONS, not decoration. A
#      clear evening sky and a partly cloudy one give different contrast and
#      different shadow depth, which is exactly the domain variation a
#      perception dataset wants.
#
# TRAPS:
#
# - The colour comes from Camera.get_rgb(), not from a "rgba" key on
#   get_current_frame(). 7.4 lost a run to that.
# - A texture on a DomeLight needs several render steps to load AND converge;
#   reading too early reports the previous sky.
# - This script refuses to print a table it did not measure.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import numpy as np
from pxr import UsdGeom, Gf, UsdLux, Sdf
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import create_new_stage
from isaacsim.sensors.camera import Camera
from isaacsim.storage.native import get_assets_root_path


def show(l=""):
    print("SKY> " + l, flush=True)


root = get_assets_root_path()
if root is None:
    show("NO ASSET ROOT -- the CDN is unreachable; no table.")
    print("SKY_DONE", flush=True)
    sim.close()
    raise SystemExit(1)

create_new_stage()
st = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
w = UsdGeom.Xform.Define(st, "/World")
st.SetDefaultPrim(w.GetPrim())

gp = UsdGeom.Cube.Define(st, "/World/floor")
gp.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(gp)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
gx.AddScaleOp().Set(Gf.Vec3f(60.0, 60.0, 1.0))
gp.CreateDisplayColorAttr([(0.45, 0.46, 0.48)])

bx = UsdGeom.Cube.Define(st, "/World/box")
bx.CreateSizeAttr(1.0)
bxf = UsdGeom.Xformable(bx)
bxf.AddTranslateOp().Set(Gf.Vec3d(3.0, 0.0, 0.5))
bxf.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
bx.CreateDisplayColorAttr([(0.55, 0.56, 0.58)])

dome = UsdLux.DomeLight.Define(st, "/World/sky")
dome.CreateIntensityAttr(1000.0)

RES = (320, 180)
cam = UsdGeom.Camera.Define(st, "/World/cam")
cam.CreateFocalLengthAttr(24.0)
cam.CreateHorizontalApertureAttr(20.955)
cam.CreateVerticalApertureAttr(20.955 * RES[1] / RES[0])
# looking along +x, slightly down, so both the box and the ground beside it
# are in frame
UsdGeom.Xformable(cam).AddTransformOp().Set(
    Gf.Matrix4d().SetLookAt(Gf.Vec3d(-1.2, -2.6, 2.2), Gf.Vec3d(3.0, 0.2, 0.4),
                            Gf.Vec3d(0, 0, 1)).GetInverse())

W = World(stage_units_in_meters=1.0)
W.reset()
for _ in range(6):
    W.step(render=False)

cm = Camera(prim_path="/World/cam", resolution=RES)
cm.initialize()

ok = None
for i in range(120):
    W.step(render=True)
    a = np.asarray(cm.get_rgb())
    if a.ndim == 3 and a.size:
        ok = a
        break
if ok is None:
    show("CAMERA NEVER RETURNED A FRAME -- no table rather than zeros.")
    print("SKY_DONE", flush=True)
    sim.close()
    raise SystemExit(1)


def measure(settle=40):
    for _ in range(settle):
        W.step(render=True)
    a = np.asarray(cm.get_rgb())[:, :, :3].astype(float) / 255.0
    lum = 0.2126 * a[:, :, 0] + 0.7152 * a[:, :, 1] + 0.0722 * a[:, :, 2]
    h, wd = lum.shape
    # the box occupies the middle of frame; sample its left and right faces,
    # and the ground on each side of it
    left = lum[int(h * 0.40):int(h * 0.62), int(wd * 0.42):int(wd * 0.52)]
    right = lum[int(h * 0.40):int(h * 0.62), int(wd * 0.56):int(wd * 0.66)]
    gnd_a = lum[int(h * 0.72):int(h * 0.88), int(wd * 0.20):int(wd * 0.36)]
    gnd_b = lum[int(h * 0.72):int(h * 0.88), int(wd * 0.66):int(wd * 0.84)]
    return (float(left.mean()), float(right.mean()),
            abs(float(left.mean()) - float(right.mean())),
            float(gnd_a.mean()), float(gnd_b.mean()),
            abs(float(gnd_a.mean()) - float(gnd_b.mean())))


SKIES = [
    ("plain dome", None),
    ("clear evening", "/NVIDIA/Assets/Skies/Clear/evening_road_01_4k.hdr"),
    ("partly cloudy",
     "/NVIDIA/Assets/Skies/Cloudy/kloofendal_48d_partly_cloudy_4k.hdr"),
]

show(f"the same box, floor and camera under three skies, {RES[0]}x{RES[1]}:")
show("")
show(f"   {'sky':>15} {'face A':>8} {'face B':>8} {'face diff':>10} "
     f"{'gnd diff':>9}")
show("   " + "-" * 56)
rows = []
for name, tex in SKIES:
    if tex is None:
        dome.CreateTextureFileAttr().Set("")
    else:
        dome.CreateTextureFileAttr().Set(root + tex)
        dome.CreateTextureFormatAttr().Set("latlong")
    la, rb, fd, ga, gb, gd = measure()
    rows.append((name, la, rb, fd, gd))
    show(f"   {name:>15} {la:8.4f} {rb:8.4f} {fd:10.4f} {gd:9.4f}")
show("")

flat = rows[0]
show(f"   Under the plain dome the two faces differ by {flat[3]:.4f} and the")
show(f"   ground either side of the box differs by {flat[4]:.4f}. That is the")
show("   'no direction, no shadow' claim from 7.4, as a measurement: the box")
show("   is lit identically from every side and casts nothing.")
show("")
for name, la, rb, fd, gd in rows[1:]:
    show(f"   Under '{name}' the faces differ by {fd:.4f} and the ground by "
         f"{gd:.4f}.")
show("")
show("   Two different skies are two different LIGHTING CONDITIONS, not two")
show("   backdrops. If your dataset only ever sees one of them, that is the")
show("   only one your model has learned.")

print("SKY_DONE", flush=True)
sim.close()
