# 3.3: references, payloads and layers. Why your scene is not one file.
#
# The lecture makes three claims and this stage has to make all three VISIBLE,
# because a reference is by nature invisible: a referenced prim looks exactly
# like a native one in the viewport. The difference only shows in the Stage tree
# (the arrow/reference badge and the non-editable subtree) and in the layer
# stack. So the stage is built so the TREE tells the story.
#
#   1. a scene is not one file      -> a component .usda is written to disk here
#                                      and the main stage only points at it
#   2. somebody else's robot gets    -> Nova Carter comes in by AddReference from
#      in without copying               a remote URL, the same call, not a copy
#   3. the file changes underneath   -> the SAME component is referenced TWICE.
#      you                              Edit the one file, both instances change.
#      That is the payoff, and it is why the two crates are deliberately not two
#      separate cubes: two cubes would prove nothing.
import omni.usd
from pxr import Usd, UsdGeom, UsdLux, Sdf, Gf

# The component the scene REFERENCES. Written as .usda (text) on purpose: 3.9
# opens this file, and a learner who can read it believes the mechanism.
#
# It goes in out/, NOT next to this script. stages/ is a host dir owned by uid
# 1000 and the container runs as 1234, so Sdf.Layer.CreateNew there fails with
#   Unable to open /workspace/stages/crate_component.usda for write
# and, worse, FAILS SILENTLY AS FAR AS THE VIEWPORT IS CONCERNED: the stage
# still opens, both references resolve to nothing, and isaac_stage_has_content
# passes on an empty pair. out/ is made group-writable by isaac_fix_log_perms.
CRATE = "/workspace/out/crate_component.usda"

# Same Carter as section 1, and the same call an asset vendor's robot arrives by.
CARTER = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
          "/Assets/Isaac/4.5/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd")


def build_component():
    """Author the crate into its own layer, on disk, as a real separate file."""
    layer = Sdf.Layer.CreateNew(CRATE)
    sub = Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(sub, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(sub, 1.0)
    # The default prim is what a reference resolves to when the URL names no
    # prim path. Without it the reference silently brings in NOTHING and the
    # stage looks like the file failed to load.
    crate = UsdGeom.Xform.Define(sub, "/crate")
    sub.SetDefaultPrim(crate.GetPrim())
    body = UsdGeom.Cube.Define(sub, "/crate/body")
    body.CreateSizeAttr(1.0)
    UsdGeom.Xformable(body).AddScaleOp().Set(Gf.Vec3f(0.6, 0.6, 0.45))
    body.CreateDisplayColorAttr([(0.55, 0.38, 0.18)])
    band = UsdGeom.Cube.Define(sub, "/crate/band")
    band.CreateSizeAttr(1.0)
    bx = UsdGeom.Xformable(band)
    bx.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.0))
    bx.AddScaleOp().Set(Gf.Vec3f(0.63, 0.63, 0.10))
    band.CreateDisplayColorAttr([(0.30, 0.22, 0.11)])
    layer.Save()
    return layer


build_component()

# Prove the component is ON DISK before anything references it. An unwritable
# path costs nothing visible at reference time: the prims exist, the subtree is
# empty, and the viewport just looks like the crates were never authored.
if not Sdf.Layer.FindOrOpen(CRATE):
    raise RuntimeError(
        f"component layer {CRATE} was not written; both crate references "
        "would resolve to an empty prim and the lecture would teach nothing")

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(ground)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
gx.AddScaleOp().Set(Gf.Vec3f(24, 24, 0.1))
ground.CreateDisplayColorAttr([(0.62, 0.64, 0.66)])

def set_translate(prim, v):
    """Set the translate op, adding one only if the prim has none.

    This is lecture 3.2's own rule, and the referenced asset is exactly why it
    exists: Carter arrives carrying its own xformOp:translate, so a bare
    AddTranslateOp here raises
      'The xformOp 'xformOp:translate' already exists'
    which reads as a bug in this script rather than as a description of what a
    reference brings with it.
    """
    x = UsdGeom.Xformable(prim)
    for op in x.GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            op.Set(v)
            return
    x.AddTranslateOp().Set(v)


# THE SAME FILE, REFERENCED TWICE. Two instances, one source. This is the pair
# that makes the "edit it once, both change" claim provable on camera: two
# separate cubes would prove nothing.
for name, x in (("crate_a", -1.3), ("crate_b", 1.3)):
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}")
    prim.GetPrim().GetReferences().AddReference(CRATE)
    set_translate(prim.GetPrim(), Gf.Vec3d(x, 0.0, 0.25))

# Somebody else's robot, by reference, from a URL. Loaded as a PAYLOAD so the
# lecture can show the difference: a payload can be unloaded, a reference cannot.
carter = UsdGeom.Xform.Define(stage, "/World/carter")
carter.GetPrim().GetPayloads().AddPayload(CARTER)
set_translate(carter.GetPrim(), Gf.Vec3d(0.0, 2.2, 0.0))

UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(1100.0)

# Report what the composition actually is, so the take can be checked against
# the runtime rather than against this file's intent (rule 5).
for path in ("/World/crate_a", "/World/crate_b", "/World/carter"):
    p = stage.GetPrimAtPath(path)
    print("COMPOSED", path, "loaded=", p.IsLoaded(), flush=True)
print("ROOT LAYER", stage.GetRootLayer().identifier, flush=True)

import asyncio
import omni.kit.app


async def _frame():
    app = omni.kit.app.get_app()
    for _ in range(700):          # the payload resolving off the network
        await app.next_update_async()
    try:
        cam = stage.GetPrimAtPath("/OmniverseKit_Persp")
        if cam:
            cam.GetAttribute("xformOp:translate").Set(Gf.Vec3d(5.2, -6.0, 3.4))
            r = (cam.GetAttribute("xformOp:rotateYXZ")
                 or cam.GetAttribute("xformOp:rotateXYZ"))
            if r:
                r.Set(Gf.Vec3f(68, 0, 41))
    except Exception as e:
        print("FRAME_FAILED", e, flush=True)


asyncio.ensure_future(_frame())
print("USD_REFERENCES_READY", flush=True)
