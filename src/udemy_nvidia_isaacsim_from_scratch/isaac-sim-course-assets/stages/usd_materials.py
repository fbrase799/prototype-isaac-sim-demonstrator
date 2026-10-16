# 3.5: meshes, materials and MDL, and why a referenced asset arrives grey.
#
# Three crates, one mesh, three material states, so the difference is a
# CONTROLLED comparison rather than three separate assertions:
#
#   /World/untextured   OmniPBR with no texture bound. The default grey.
#   /World/broken       OmniPBR whose albedo path is relative to the ASSET ROOT,
#                       which is the wrong base. The texture is not found.
#   /World/textured     the same material with the path resolved correctly.
#
# The texture is GENERATED here rather than shipped, so the repo carries no
# binary and the lecture can state exactly what the image is: a 256x256 checker
# with a painted stripe, which makes a wrong UV or a missing bind obvious.
import os

import omni.usd
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf

# out/, not stages/: stages/ is a host dir owned by uid 1000 and this container
# runs as 1234. A failed write there is silent and the reference resolves to
# nothing (see 3.3 and 3.4).
OUT = "/workspace/out"
MATERIALS_DIR = f"{OUT}/materials"          # where the .mdl and textures live
TEX_NAME = "crate_albedo.png"
TEX_PATH = f"{MATERIALS_DIR}/{TEX_NAME}"


def write_texture():
    """A checker with a stripe, written with PIL if present, else raw PNG.

    Deliberately not a photograph: a checker shows UV orientation and tiling,
    and a solid colour would look identical to a material that failed to bind.
    """
    os.makedirs(MATERIALS_DIR, exist_ok=True)
    try:
        from PIL import Image
        im = Image.new("RGB", (256, 256))
        px = im.load()
        for y in range(256):
            for x in range(256):
                c = ((x // 32) + (y // 32)) % 2
                px[x, y] = (150, 104, 48) if c else (110, 74, 32)
        for y in range(112, 144):           # the stripe: orientation is visible
            for x in range(256):
                px[x, y] = (196, 148, 70)
        im.save(TEX_PATH)
        return True
    except Exception as e:
        print("TEXTURE_FAILED", e, flush=True)
        return False


ok = write_texture()
if not ok or not os.path.exists(TEX_PATH):
    raise RuntimeError(f"{TEX_PATH} was not written; every crate would be grey "
                       "and the lecture could not tell a missing texture from a "
                       "missing FILE")
print("FACT texture", TEX_PATH, os.path.getsize(TEX_PATH), "bytes", flush=True)

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(ground)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
gx.AddScaleOp().Set(Gf.Vec3f(30, 30, 0.1))
ground.CreateDisplayColorAttr([(0.62, 0.64, 0.66)])

looks = UsdGeom.Scope.Define(stage, "/World/Looks")


def make_material(name, albedo_path):
    """An OmniPBR material, optionally with a diffuse texture bound.

    albedo_path None means "no texture at all", which is the honest grey.
    A path that does not resolve is a DIFFERENT failure and looks the same,
    which is the whole point of the lecture.
    """
    mat = UsdShade.Material.Define(stage, f"/World/Looks/{name}")
    shader = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/Shader")
    shader.CreateIdAttr("OmniPBR")
    shader.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    shader.CreateInput("diffuse_color_constant",
                       Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.55, 0.38, 0.18))
    if albedo_path is not None:
        shader.CreateInput("diffuse_texture",
                           Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(albedo_path))
    # The surface output MUST be connected in the "mdl" render context, and it
    # must be connected to the shader's own "out" OUTPUT, which has to exist
    # first. Probed on the first version: inputs were all set correctly and
    # `GetSurfaceOutput("mdl")` still read None, so MDL never evaluated and every
    # crate rendered the flat diffuse_color_constant. A texture that is bound but
    # never evaluated looks exactly like a texture that failed to load, which is
    # the confusion this lecture is about, so getting it right matters twice.
    # Connect by passing the OUTPUT OBJECT, not (connectable, name).
    #
    # `ConnectToSource(shader.ConnectableAPI(), "out")` is the form the USD docs
    # show and it silently does nothing here: probed afterwards,
    # GetSurfaceOutput("mdl") still read None with every input set correctly, so
    # MDL never evaluated and all three crates rendered the flat
    # diffuse_color_constant. This is the form Kit's own material_library.py
    # uses (material_library.py:431-434), and it is the one that takes.
    shader_out = shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(shader_out)
    return mat


def crate(path, x, mat):
    """A UV-MAPPED MESH, not a UsdGeom.Cube.

    A Cube carries no `st` primvar (probed: it has only displayColor and
    displayOpacity), so there is nothing for a texture to map onto and OmniPBR
    falls back to diffuse_color_constant. Every crate then renders flat and a
    working texture is indistinguishable from a broken one, which would make
    this lecture's central comparison meaningless.

    So the crate is authored as a mesh with explicit points, faces and per-face
    UVs. That is also honest about the subject: a mesh is points plus topology
    plus primvars, and the texture coordinate is one of those primvars.
    """
    h = 0.3
    pts = [(-h, -h, -h), (h, -h, -h), (h, h, -h), (-h, h, -h),
           (-h, -h,  h), (h, -h,  h), (h, h,  h), (-h, h,  h)]
    faces = [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1),
             (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    m.CreateFaceVertexCountsAttr([4] * len(faces))
    m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    # one full 0..1 square per face, so the checker reads at the same scale
    # everywhere and a rotated UV would be obvious
    st_vals = [Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)] * len(faces)
    stv = UsdGeom.PrimvarsAPI(m.GetPrim()).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    stv.Set(st_vals)
    UsdGeom.Xformable(m).AddTranslateOp().Set(Gf.Vec3d(x, 0.0, 0.3))
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    return m


# 1. no texture bound at all: the honest grey
crate("/World/untextured", -1.5, make_material("Untextured", None))

# 2. THE BUG. The path is relative to the ASSET ROOT, which is the wrong base.
#    A texture referenced as "./textures/crate_albedo.png" from a material that
#    lives in materials/ resolves against materials/, not against the root, so
#    this points at a file that does not exist. Nothing errors in the viewport.
crate("/World/broken", 0.0, make_material("Broken", f"./textures/{TEX_NAME}"))

# 3. resolved correctly, as an absolute path so the comparison is unambiguous
crate("/World/textured", 1.5, make_material("Textured", TEX_PATH))

UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(1200.0)

for p in ("/World/untextured", "/World/broken", "/World/textured"):
    prim = stage.GetPrimAtPath(p)
    b = UsdShade.MaterialBindingAPI(prim).GetDirectBinding()
    print("FACT bind", p, b.GetMaterialPath(), flush=True)

import asyncio
import omni.kit.app


async def _frame():
    """Own camera, look-at matrix, viewport repointed. See usd_units.py: the
    default camera prim cannot be written reliably and hand-solved Euler angles
    put the camera inside the subject."""
    app = omni.kit.app.get_app()
    for _ in range(700):
        await app.next_update_async()
    try:
        import omni.kit.viewport.utility as vpu
        cam = UsdGeom.Camera.Define(stage, "/World/lecture_cam")
        cam.CreateFocalLengthAttr(28.0)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 10000.0))
        xf = UsdGeom.Xformable(cam.GetPrim())
        # Back far enough for all THREE crates: they span x -1.5 to +1.5, and
        # the first framing cropped the outer two at both edges.
        m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(1.1, -5.2, 2.4),
                                    Gf.Vec3d(0.0, 0.0, 0.30),
                                    Gf.Vec3d(0, 0, 1)).GetInverse()
        xf.AddTransformOp().Set(m)
        vpu.get_active_viewport().camera_path = "/World/lecture_cam"
        print("FACT cam_set /World/lecture_cam", flush=True)
    except Exception as e:
        print("FRAME_FAILED", e, flush=True)


asyncio.ensure_future(_frame())
print("USD_MATERIALS_READY", flush=True)
