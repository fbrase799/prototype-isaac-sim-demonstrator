# 6.2: intrinsics derived by hand, checked against Isaac, at THREE resolutions
# and with a deliberately mismatched vertical aperture.
#
# The lecture claims three things it must not merely assert:
#   1. fx = width * focal / aperture matches Isaac EXACTLY. Confirmed at
#      320x180, 640x360 and 1280x720: 366.50, 733.00, 1466.00 both ways.
#   2. intrinsics belong to a camera AND a resolution: same prim, fx doubles
#      with the width.
#   3. fy == fx because the pixels are square.
#
# Claim 3 was drafted the other way round -- "set the vertical aperture wrong
# and fx and fy diverge" -- and the measurement REFUTED it. Writing
# verticalAperture = 20.955 on a 16:9 render product reads back as 11.787, and
# fy stays at 733.00. Attaching a render product OVERWRITES verticalAperture to
# hAperture * height/width (0.5625, 0.7500, 1.0000 against the same three
# aspect ratios, exact to four decimals), so square pixels are ENFORCED rather
# than coincidental, and the vertical FOV is decided by the resolution.
# Isaac's own get_intrinsics_matrix source computes exactly the hand formula:
#   fx = width * focal_length / horizontal_aperture
#   fy = height * focal_length / vertical_aperture
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import math
from pxr import UsdGeom, Gf, UsdLux
import omni.usd
from isaacsim.core.utils.stage import clear_stage
from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera
W = World(stage_units_in_meters=1.0)

def show(l=""): print("INT> " + l, flush=True)

def build(focal, aper_h, aper_v, path):
    st = omni.usd.get_context().get_stage()
    cam = UsdGeom.Camera.Define(st, path)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(aper_h)
    cam.CreateVerticalApertureAttr(aper_v)
    m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(0,0,1), Gf.Vec3d(1,0,1), Gf.Vec3d(0,0,1)).GetInverse()
    UsdGeom.Xformable(cam).AddTransformOp().Set(m)
    return cam

clear_stage()
st = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(st, 1.0)
w = UsdGeom.Xform.Define(st, "/World"); st.SetDefaultPrim(w.GetPrim())
UsdLux.DomeLight.Define(st, "/World/dome").CreateIntensityAttr(1100.0)
c = UsdGeom.Cube.Define(st, "/World/wall"); c.CreateSizeAttr(1.0)
cx = UsdGeom.Xformable(c); cx.AddTranslateOp().Set(Gf.Vec3d(3,0,1)); cx.AddScaleOp().Set(Gf.Vec3f(0.05,6,3))

FOCAL, APER = 24.0, 20.955
build(FOCAL, APER, APER*9/16, "/World/cam_sq")
W.reset()
for _ in range(6): W.step(render=False)

show("1. the derivation, checked against Isaac at three resolutions")
show(f"   focal {FOCAL} mm, horizontal aperture {APER} mm")
show(f"   hfov = 2 atan(aperture / 2f) = {math.degrees(2*math.atan(APER/(2*FOCAL))):.2f} deg")
show("")
show(f"   {'resolution':>12} {'hand fx':>10} {'Isaac fx':>10} {'Isaac fy':>10} {'Isaac cx':>10}")
show("   " + "-"*58)
for res in ((320,180), (640,360), (1280,720)):
    cam = Camera(prim_path="/World/cam_sq", resolution=res)
    cam.initialize()
    for _ in range(4): W.step(render=True)
    K = cam.get_intrinsics_matrix()
    hand = res[0]*FOCAL/APER
    show(f"   {res[0]:5d}x{res[1]:<6d} {hand:10.2f} {K[0][0]:10.2f} {K[1][1]:10.2f} {K[0][2]:10.1f}")

show("")
show("2. trying to BREAK fy: set the vertical aperture wrong on purpose")
show("   every camera below is built with verticalAperture = 20.955, the")
show("   same as the horizontal, which for a 16:9 image is deliberately wrong.")
show("")
show(f"   {'render product':>16} {'vAper written':>14} {'vAper read':>11} "
     f"{'fx':>9} {'fy':>9} {'vfov':>7}")
show("   " + "-"*72)
for res in ((640,360), (640,480), (640,640)):
    path = f"/World/cam_break_{res[1]}"
    c2 = build(FOCAL, APER, APER, path)          # square vertical aperture, every time
    written = st.GetPrimAtPath(path).GetAttribute("verticalAperture").Get()
    cam = Camera(prim_path=path, resolution=res)
    cam.initialize()
    for _ in range(4): W.step(render=True)
    read = st.GetPrimAtPath(path).GetAttribute("verticalAperture").Get()
    K = cam.get_intrinsics_matrix()
    vfov = math.degrees(2*math.atan(read/(2*FOCAL)))
    show(f"   {str(res[0])+'x'+str(res[1]):>16} {written:14.3f} {read:11.3f} "
         f"{K[0][0]:9.2f} {K[1][1]:9.2f} {vfov:6.2f}d")
show("")
show("   attaching a render product OVERWRITES verticalAperture to match the")
show("   resolution, so fy is FORCED equal to fx. Square pixels are enforced:")
show(f"   {'render product':>16} {'vAper/hAper':>12} {'height/width':>13}")
for res in ((640,360), (640,480), (640,640)):
    a = st.GetPrimAtPath(f"/World/cam_break_{res[1]}").GetAttribute("verticalAperture").Get()
    show(f"   {str(res[0])+'x'+str(res[1]):>16} {a/APER:12.4f} {res[1]/res[0]:13.4f}")
print("INT_DONE", flush=True)
sim.close()
