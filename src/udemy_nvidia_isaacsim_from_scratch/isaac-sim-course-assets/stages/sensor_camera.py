# Section 6: camera intrinsics and depth, measured from a real render.
#
# Two results this section is built on, both checked against hand arithmetic:
#
# 1. INTRINSICS. fx = width * focal / horizontal_aperture. At 24 mm on a
#    20.955 mm aperture at 640x360 that is 733.00 px, and Isaac reports
#    fx = 733.00, fy = 733.00, cx = 320.0. The hfov is 47.17 degrees.
#
# 2. DEPTH IS NOT DISTANCE, and the gap is measurable on a FLAT WALL:
#      centre  image-plane 2.975 m   to-camera 2.975 m   diff 0.000
#      edge    image-plane 2.975 m   to-camera 3.187 m   diff 0.212
#    Same wall, same render. distance_to_image_plane is the perpendicular
#    distance (what a depth camera calls Z); distance_to_camera is along the
#    ray. Checked: the pixel at 94% width is 20.98 deg off axis, and
#    2.975 / cos(20.98) = 3.186 against a measured 3.187, so they agree to
#    0.9 mm. That factor of cos(theta) IS lecture 6.3.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import math, numpy as np
from pxr import UsdGeom, UsdPhysics, Gf, UsdLux
import omni.usd
from isaacsim.core.utils.stage import clear_stage
from isaacsim.core.api import World
W=World(stage_units_in_meters=1.0)

def show(l=""): print("CAM> "+l, flush=True)

clear_stage()
st=omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(st,UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(st,1.0)
w=UsdGeom.Xform.Define(st,"/World"); st.SetDefaultPrim(w.GetPrim())
UsdLux.DomeLight.Define(st,"/World/dome").CreateIntensityAttr(1100.0)
# A wall straight ahead at a KNOWN distance, and a second one off to the side,
# so depth-along-the-ray and depth-along-the-axis can differ measurably.
for name,(x,y,z),(sx,sy,sz) in (("wall",(3.0,0,1.0),(0.05,6.0,3.0)),):
    c=UsdGeom.Cube.Define(st,f"/World/{name}"); c.CreateSizeAttr(1.0)
    cx=UsdGeom.Xformable(c); cx.AddTranslateOp().Set(Gf.Vec3d(x,y,z)); cx.AddScaleOp().Set(Gf.Vec3f(sx,sy,sz))
    c.CreateDisplayColorAttr([(0.62,0.64,0.66)])

cam=UsdGeom.Camera.Define(st,"/World/cam")
FOCAL=24.0; APER=20.955
cam.CreateFocalLengthAttr(FOCAL)
cam.CreateHorizontalApertureAttr(APER)
cam.CreateVerticalApertureAttr(APER*9/16)
m=Gf.Matrix4d().SetLookAt(Gf.Vec3d(0,0,1.0),Gf.Vec3d(1,0,1.0),Gf.Vec3d(0,0,1)).GetInverse()
UsdGeom.Xformable(cam).AddTransformOp().Set(m)

W.reset()
for _ in range(6): W.step(render=False)

from isaacsim.sensors.camera import Camera
RES=(640,360)
c = Camera(prim_path="/World/cam", resolution=RES)
c.initialize()
c.add_distance_to_image_plane_to_frame()
c.add_distance_to_camera_to_frame()
for _ in range(12): W.step(render=True)

show("1. intrinsics, derived from the aperture and focal length")
hfov = 2*math.atan(APER/(2*FOCAL))
fx = RES[0]*FOCAL/APER
show(f"   focal {FOCAL} mm, horizontal aperture {APER} mm, {RES[0]}x{RES[1]}")
show(f"   hfov = 2 atan(aperture / 2f) = {math.degrees(hfov):.2f}°")
show(f"   fx   = width * f / aperture  = {fx:.2f} px")
try:
    K = c.get_intrinsics_matrix()
    show(f"   Isaac reports fx = {K[0][0]:.2f}, fy = {K[1][1]:.2f}, cx = {K[0][2]:.1f}")
except Exception as e:
    show(f"   get_intrinsics_matrix failed: {type(e).__name__}")

show("")
show("2. depth: distance to the IMAGE PLANE vs distance to the CAMERA")
f = c.get_current_frame()
dip = f.get("distance_to_image_plane"); dtc = f.get("distance_to_camera")
if dip is None or dtc is None:
    show(f"   annotators returned None (keys: {list(f.keys())})")
else:
    dip=np.asarray(dip); dtc=np.asarray(dtc)
    h,wd = dip.shape[:2]
    for lab,(row,col) in (("centre",(h//2,wd//2)),("edge",(h//2,int(wd*0.94)))):
        a=float(dip[row,col]); b=float(dtc[row,col])
        show(f"   {lab:8s} image-plane {a:6.3f} m   to-camera {b:6.3f} m   diff {b-a:+6.3f}")
    show("")
    show(f"   the wall is at x = 3.0 m and the camera at x = 0.0")
print("CAM_DONE", flush=True)
sim.close()
