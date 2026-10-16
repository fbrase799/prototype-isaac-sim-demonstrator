# Build a small populated stage: ground plane, a few shaded boxes and cylinders,
# a dome light. Purpose is calibration footage that resembles a real lecture
# viewport (lit surfaces, structure, shadows) rather than an empty grid.
from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, Gf, Sdf
import omni.usd

stage = omni.usd.get_context().get_stage()
root = UsdGeom.Xform.Define(stage, "/World/cal")

# ground
plane = UsdGeom.Mesh.Define(stage, "/World/cal/ground")
S = 12.0
plane.CreatePointsAttr([(-S,-S,0),(S,-S,0),(S,S,0),(-S,S,0)])
plane.CreateFaceVertexCountsAttr([4])
plane.CreateFaceVertexIndicesAttr([0,1,2,3])
plane.CreateDisplayColorAttr([(0.32,0.34,0.38)])

# a row of boxes at varying heights, so the frame has real structure
for i in range(7):
    c = UsdGeom.Cube.Define(stage, f"/World/cal/box{i}")
    c.CreateSizeAttr(1.0)
    x = -5.4 + i*1.8
    h = 0.5 + (i % 4) * 0.55
    UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(x, 1.2, h))
    c.CreateDisplayColorAttr([(0.45 + 0.07*(i%3), 0.52, 0.62)])

# cylinders for curvature/specular variety
for i in range(4):
    cy = UsdGeom.Cylinder.Define(stage, f"/World/cal/cyl{i}")
    cy.CreateRadiusAttr(0.45); cy.CreateHeightAttr(1.8)
    UsdGeom.Xformable(cy).AddTranslateOp().Set(Gf.Vec3d(-4.5 + i*3.0, -2.7, 0.9))
    cy.CreateDisplayColorAttr([(0.70, 0.62, 0.30)])

# light: unrotated dome (rotating it tips the sky sideways on a Z-up stage)
dome = UsdLux.DomeLight.Define(stage, "/World/cal/dome")
dome.CreateIntensityAttr(1100.0)
# Point the viewport camera at the geometry. Without this the default camera
# looks past a 12 m stage and the frame is bare ground, which is indistinguishable
# from "the script did not run".
try:
    import omni.kit.viewport.utility as vp_util
    vp = vp_util.get_active_viewport()
    cam_path = "/World/cal/calCam"
    cam = UsdGeom.Camera.Define(stage, cam_path)
    xf = UsdGeom.Xformable(cam)
    xf.AddTranslateOp().Set(Gf.Vec3d(9.0, -11.0, 5.0))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(74.0, 0.0, 38.0))
    if vp is not None:
        vp.camera_path = cam_path
    print("[cal] camera framed")
except Exception as exc:
    print("[cal] camera framing skipped:", exc)

print("[cal] stage populated:", len(list(stage.Traverse())), "prims")
