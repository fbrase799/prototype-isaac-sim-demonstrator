# 6.3: depth is not distance, measured on a FLAT WALL.
#
# Two annotators, same render, same pixel:
#   distance_to_image_plane  -- perpendicular distance, what a depth camera
#                               calls Z, what a reconstruction wants
#   distance_to_camera       -- length of the ray, what "how far away is it"
#                               sounds like it should mean
# On a flat wall these differ by exactly 1/cos(theta), and theta grows toward
# the edge of frame. That factor IS the lecture.
#
# TWO TRAPS, both hit while building this:
#
# 1. THE ANNOTATORS RETURN NOTHING IF THE KIT CACHE IS ROOT-OWNED. The keys are
#    present in the frame dict and the VALUES are None, forever, however many
#    render steps you take; going straight to rep.AnnotatorRegistry gives a
#    (0,) array instead. The only symptom is two lines buried in Kit's startup:
#    "Failed to initialize rtx::shaderdb::ContextManager". Fix, once per
#    container:
#        docker exec -u 0 isaac-sim chown -R isaac-sim:isaac-sim /isaac-sim/kit/cache
#    RGB renders fine throughout, so every capture take still looks healthy.
#
# 2. So this script STEPS UNTIL THE DATA IS ACTUALLY 2-D and prints
#    "ANNOTATORS NEVER POPULATED" rather than a table if it is not. An earlier
#    version used f.get(...) with a fallback and would happily have printed a
#    table of zeros as though it had measured something.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import math
import numpy as np
from pxr import UsdGeom, Gf, UsdLux
import omni.usd
from isaacsim.core.utils.stage import clear_stage
from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera
W = World(stage_units_in_meters=1.0)


def show(l=""):
    print("DEP> " + l, flush=True)


clear_stage()
st = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
w = UsdGeom.Xform.Define(st, "/World")
st.SetDefaultPrim(w.GetPrim())
UsdLux.DomeLight.Define(st, "/World/dome").CreateIntensityAttr(1100.0)

# A FLAT wall, perpendicular to the optical axis, at a known distance. Flat is
# the whole point: any difference between the two annotators is geometry, not
# scene content.
WALLX = 3.0
c = UsdGeom.Cube.Define(st, "/World/wall")
c.CreateSizeAttr(1.0)
cx = UsdGeom.Xformable(c)
cx.AddTranslateOp().Set(Gf.Vec3d(WALLX, 0, 1.0))
cx.AddScaleOp().Set(Gf.Vec3f(0.05, 8.0, 4.0))
c.CreateDisplayColorAttr([(0.62, 0.64, 0.66)])

FOCAL, APER = 24.0, 20.955
RES = (640, 360)
cam = UsdGeom.Camera.Define(st, "/World/cam")
cam.CreateFocalLengthAttr(FOCAL)
cam.CreateHorizontalApertureAttr(APER)
cam.CreateVerticalApertureAttr(APER * RES[1] / RES[0])
UsdGeom.Xformable(cam).AddTransformOp().Set(
    Gf.Matrix4d().SetLookAt(Gf.Vec3d(0, 0, 1.0), Gf.Vec3d(1, 0, 1.0),
                            Gf.Vec3d(0, 0, 1)).GetInverse())

W.reset()
for _ in range(6):
    W.step(render=False)
cm = Camera(prim_path="/World/cam", resolution=RES)
cm.initialize()
cm.add_distance_to_image_plane_to_frame()
cm.add_distance_to_camera_to_frame()

dip = dtc = None
for i in range(160):
    W.step(render=True)
    f = cm.get_current_frame()
    a = np.asarray(f.get("distance_to_image_plane")
                   if f.get("distance_to_image_plane") is not None else [])
    b = np.asarray(f.get("distance_to_camera")
                   if f.get("distance_to_camera") is not None else [])
    if a.ndim == 2 and b.ndim == 2 and a.size and b.size:
        dip, dtc = a, b
        show(f"annotators populated after {i + 1} render steps")
        break
if dip is None:
    show("ANNOTATORS NEVER POPULATED -- no table, rather than a table of zeros")
    show("check: docker exec -u 0 isaac-sim chown -R isaac-sim:isaac-sim /isaac-sim/kit/cache")
    print("DEP_DONE", flush=True)
    sim.close()
    raise SystemExit(1)

h, wd = dip.shape[:2]
fx = RES[0] * FOCAL / APER
row = h // 2

show(f"a FLAT wall at x = {WALLX:.2f} m, camera at x = 0.00, "
     f"{RES[0]}x{RES[1]}, fx = {fx:.2f} px")
show("both annotators read on the SAME render, along the centre row")
show("")
show(f"   {'column':>8} {'off axis':>9} {'image plane':>12} {'to camera':>11} "
     f"{'ratio':>8} {'1/cos':>8}")
show("   " + "-" * 64)
for col in (wd // 2, int(wd * 0.60), int(wd * 0.75), int(wd * 0.88), wd - 1):
    a = float(dip[row, col])
    b = float(dtc[row, col])
    theta = math.atan((col - wd / 2 + 0.5) / fx)
    show(f"   {col:8d} {math.degrees(theta):8.2f}d {a:12.3f} {b:11.3f} "
         f"{b / a:8.4f} {1 / math.cos(theta):8.4f}")
show("")
show("   the ratio IS 1/cos(theta) to four decimals: same wall, same render,")
show("   two correct answers to 'how far away is it'")
show("")

# The standard point-cloud recipe is X = (ray scaled so its forward component
# is 1) * Z. There are two ways to get it wrong and they bow the wall in
# OPPOSITE directions, which is useful diagnostically: a wall bulging toward
# you and one sagging away from you are different bugs in the same line.
show("what the wrong choice costs: rebuild the flat wall three ways")
show(f"   {'off axis':>9} {'correct':>9} {'range as Z':>11} {'err':>8} "
     f"{'unit ray x Z':>13} {'err':>8}")
show("   " + "-" * 66)
for col in (wd // 2, int(wd * 0.60), int(wd * 0.75), wd - 1):
    theta = math.atan((col - wd / 2 + 0.5) / fx)
    a = float(dip[row, col])
    b = float(dtc[row, col])
    right = a                          # forward component is exactly Z
    wrong_range = b                    # feeding the RANGE into the same recipe
    wrong_unit = a * math.cos(theta)   # scaling a UNIT ray by Z instead
    show(f"   {math.degrees(theta):8.2f}d {right:9.3f} {wrong_range:11.3f} "
         f"{wrong_range - right:+8.3f} {wrong_unit:13.3f} "
         f"{wrong_unit - right:+8.3f}")
show("")
th_e = math.atan((wd - 1 - wd / 2 + 0.5) / fx)
e_far = float(dtc[row, wd - 1]) - float(dip[row, wd - 1])
e_near = float(dip[row, wd - 1]) * math.cos(th_e) - float(dip[row, wd - 1])
show(f"   a FLAT wall bows {e_far:+.3f} m one way or {e_near:+.3f} m the")
show("   other at the frame edge. Neither raises an error.")
print("DEP_DONE", flush=True)
sim.close()
