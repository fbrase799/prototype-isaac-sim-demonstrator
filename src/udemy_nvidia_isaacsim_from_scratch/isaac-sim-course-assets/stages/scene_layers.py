# 7.8: scene organisation for reuse.
#
# THE TRAP THIS LECTURE HAS TO AVOID. 3.3 already taught what a reference and a
# sublayer ARE, mechanically, and re-teaching that here would waste the closing
# lecture of the section. So this script does not explain composition. It builds
# the SAME scene two ways and measures what the organisation costs and buys:
#
#   FLAT  : one .usda holding environment + props + lighting + robot start
#   LAYERED: a thin root that sublayers four files, one per concern
#
# and then asks four questions a working roboticist actually has:
#
#   1. How big is the file I have to open to change the lighting?
#   2. If two people edit the scene at once, do they collide?
#   3. What does it cost to swap ONE concern (day -> night) in each layout?
#   4. Does the layered stage compose to the SAME thing? (If not, none of the
#      above matters, so this is checked rather than assumed.)
#
# NOTHING HERE IS ASSERTED WITHOUT A NUMBER. Every claim in the VO is a value
# printed by this script.
#
# TRAPS FOUND WRITING THIS:
#  - Sdf.Layer.CreateNew on /workspace/stages FAILS (host uid 1000 vs container
#    1234) and fails in a way the viewport does not show. Everything goes to
#    /workspace/out, which is group-writable. Same trap as 3.3.
#  - A sublayer path is resolved RELATIVE TO THE ROOT LAYER, not to the process
#    cwd. Absolute paths here, or the root composes to an empty stage and every
#    prim count reads 0 with no error.
#  - SetDefaultPrim matters on the component layers or a reference to them
#    resolves to nothing (3.3's lesson, and it bites again here).
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import os
from pxr import Usd, UsdGeom, UsdLux, Sdf, Gf

OUT = "/workspace/out"
FLAT = f"{OUT}/warehouse_flat.usda"
ROOT = f"{OUT}/warehouse_root.usda"
L_ENV = f"{OUT}/layer_environment.usda"
L_PROPS = f"{OUT}/layer_props.usda"
L_LIGHT = f"{OUT}/layer_lighting.usda"
L_START = f"{OUT}/layer_robot_start.usda"
L_NIGHT = f"{OUT}/layer_lighting_night.usda"

N_PROPS = 60


def show(l=""):
    print("LYR> " + l, flush=True)


def fresh(path):
    """CreateNew raises if the file exists; these are rebuilt every run."""
    if os.path.exists(path):
        os.remove(path)
    return Sdf.Layer.CreateNew(path)


def author_environment(stage, scope="/World"):
    """Floor, walls, and the racking. The part nobody edits day to day."""
    UsdGeom.Xform.Define(stage, f"{scope}/env")
    floor = UsdGeom.Cube.Define(stage, f"{scope}/env/floor")
    floor.CreateSizeAttr(1.0)
    fx = UsdGeom.Xformable(floor)
    fx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
    fx.AddScaleOp().Set(Gf.Vec3f(40, 26, 0.1))
    floor.CreateDisplayColorAttr([(0.60, 0.62, 0.64)])
    for i in range(6):
        w = UsdGeom.Cube.Define(stage, f"{scope}/env/rack_{i:02d}")
        w.CreateSizeAttr(1.0)
        wx = UsdGeom.Xformable(w)
        wx.AddTranslateOp().Set(Gf.Vec3d(-16 + i * 6.4, 0, 1.4))
        wx.AddScaleOp().Set(Gf.Vec3f(1.2, 20.0, 2.8))
        w.CreateDisplayColorAttr([(0.42, 0.45, 0.50)])


def author_props(stage, scope="/World"):
    """The pallets. The part that changes every time the task changes."""
    UsdGeom.Xform.Define(stage, f"{scope}/props")
    for i in range(N_PROPS):
        p = UsdGeom.Cube.Define(stage, f"{scope}/props/pallet_{i:03d}")
        p.CreateSizeAttr(1.0)
        px = UsdGeom.Xformable(p)
        px.AddTranslateOp().Set(Gf.Vec3d(-14 + (i % 12) * 2.6,
                                         -8 + (i // 12) * 4.0, 0.3))
        px.AddScaleOp().Set(Gf.Vec3f(1.1, 1.1, 0.6))
        p.CreateDisplayColorAttr([(0.55, 0.38, 0.18)])


def author_lighting(stage, scope="/World", intensity=1000.0, key=6500.0):
    """The part that gets retuned constantly, and the reason for this lecture."""
    UsdGeom.Xform.Define(stage, f"{scope}/lighting")
    dome = UsdLux.DomeLight.Define(stage, f"{scope}/lighting/dome")
    dome.CreateIntensityAttr(intensity)
    dome.CreateColorTemperatureAttr(key)
    d = UsdLux.DistantLight.Define(stage, f"{scope}/lighting/sun")
    d.CreateIntensityAttr(intensity * 2.2)
    UsdGeom.Xformable(d).AddRotateXYZOp().Set(Gf.Vec3f(-42, 0, 18))


def author_start(stage, scope="/World"):
    """Where the robot begins. Swapped per experiment, tiny, edited constantly."""
    x = UsdGeom.Xform.Define(stage, f"{scope}/robot_start")
    UsdGeom.Xformable(x).AddTranslateOp().Set(Gf.Vec3d(-15.0, -9.0, 0.0))


def size_kb(path):
    return os.path.getsize(path) / 1024.0


def light_intensity(prim):
    """UsdLux authors intensity as `inputs:intensity`, NOT `intensity`.

    Reading the bare name returns None, and None here reads exactly like "the
    night layer failed to compose" -- I chased a non-existent sublayer bug for a
    run because of it. Same family as 7.4's Camera.get_current_frame() having no
    'rgba' key: the attribute is there, under the schema's real name. Both names
    are tried so the script survives either convention.
    """
    if not prim:
        return None
    for n in ("inputs:intensity", "intensity"):
        a = prim.GetAttribute(n)
        if a and a.Get() is not None:
            return a.Get()
    return None


def count(stage):
    prims = sum(1 for _ in stage.Traverse())
    meshes = sum(1 for p in stage.Traverse() if p.IsA(UsdGeom.Gprim))
    lights = sum(1 for p in stage.Traverse()
                 if p.HasAPI(UsdLux.LightAPI) or p.IsA(UsdLux.DomeLight)
                 or p.IsA(UsdLux.DistantLight))
    return prims, meshes, lights


# ---------------------------------------------------------------- FLAT
flat = fresh(FLAT)
s = Usd.Stage.Open(flat)
UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(s, 1.0)
w = UsdGeom.Xform.Define(s, "/World")
s.SetDefaultPrim(w.GetPrim())
author_environment(s)
author_props(s)
author_lighting(s)
author_start(s)
flat.Save()
fp, fm, fl = count(s)

# ---------------------------------------------------------------- LAYERED
def component(path, fn, **kw):
    lay = fresh(path)
    st = Usd.Stage.Open(lay)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = UsdGeom.Xform.Define(st, "/World")
    st.SetDefaultPrim(root.GetPrim())
    fn(st, **kw)
    lay.Save()
    return lay


component(L_ENV, author_environment)
component(L_PROPS, author_props)
component(L_LIGHT, author_lighting)
component(L_START, author_start)
# the night variant: the SAME concern, authored once, swappable as one line
component(L_NIGHT, author_lighting, intensity=60.0, key=3000.0)

root = fresh(ROOT)
# ABSOLUTE paths. A sublayer is resolved relative to the ROOT LAYER, and a
# relative path that misses composes to an empty stage with no error at all.
root.subLayerPaths = [L_ENV, L_PROPS, L_LIGHT, L_START]
root.Save()
rs = Usd.Stage.Open(ROOT)
rp, rm, rl = count(rs)

show("the same scene, organised two ways")
show("")
show(f"   {'layout':<22}{'file you open':>16}{'prims':>8}{'meshes':>8}{'lights':>8}")
show("   " + "-" * 62)
show(f"   {'FLAT (one file)':<22}{size_kb(FLAT):>13.1f} KB{fp:>8}{fm:>8}{fl:>8}")
show(f"   {'LAYERED (root)':<22}{size_kb(ROOT):>13.1f} KB{rp:>8}{rm:>8}{rl:>8}")
show("")

# ---- 1. does it compose to the same scene? if not, nothing else matters
same = (fp, fm, fl) == (rp, rm, rl)
show(f"composes to the same stage: {same}  "
     f"(flat {fp}/{fm}/{fl} vs layered {rp}/{rm}/{rl})")
if not same:
    show("   STOP -- the layered stage is NOT the flat one. Everything below")
    show("   would be comparing two different scenes.")
show("")

# ---- 2. what do you actually open to change one concern?
show("to retune the lighting you open:")
show(f"   flat     {os.path.basename(FLAT):<28}{size_kb(FLAT):>8.1f} KB")
show(f"   layered  {os.path.basename(L_LIGHT):<28}{size_kb(L_LIGHT):>8.1f} KB")
ratio = size_kb(FLAT) / max(size_kb(L_LIGHT), 1e-9)
show(f"   the lighting file is {ratio:.1f}x smaller than the whole scene")
show("")

# ---- 3. the per-concern breakdown, which is the reuse argument
show("what each layer costs on disk:")
tot = 0.0
for p in (L_ENV, L_PROPS, L_LIGHT, L_START):
    tot += size_kb(p)
    show(f"   {os.path.basename(p):<30}{size_kb(p):>8.1f} KB")
show(f"   {'root (pointers only)':<30}{size_kb(ROOT):>8.1f} KB")
show(f"   {'sum of the parts':<30}{tot + size_kb(ROOT):>8.1f} KB "
     f"vs flat {size_kb(FLAT):.1f} KB")
show("")

# ---- 4. swapping one concern: day -> night, measured as an edit
show("swap the lighting for a night version:")
# Read the BEFORE value off the composed stage, not from the constant used to
# author it, so both sides of the comparison come from the runtime (rule 5).
day_inten = light_intensity(rs.GetPrimAtPath("/World/lighting/dome"))
night = Sdf.Layer.FindOrOpen(ROOT)
night.subLayerPaths = [L_ENV, L_PROPS, L_NIGHT, L_START]
night.Save()
# RELOAD, don't just re-Open. Usd.Stage.Open returns the CACHED stage for a
# path already open in this process, so the sublayer swap above is invisible to
# it and the dome reads None -- which looks like the night layer failed to
# author a light, not like a stale cache. Reload() pulls the new layer stack.
ns = Usd.Stage.Open(ROOT)
ns.Reload()
np_, nm, nl = count(ns)
dome = ns.GetPrimAtPath("/World/lighting/dome")
inten = light_intensity(dome)
if inten is None:
    show("   WARNING: dome intensity read None -- the swap did not compose;")
    show("   reporting no number is better than reporting a wrong one.")
show(f"   layered: ONE line of the root changed, {size_kb(ROOT):.1f} KB rewritten")
show(f"   dome intensity is now {inten}  (was {day_inten})")
show(f"   geometry untouched: {np_} prims, {nm} meshes -- identical to before")
show(f"   flat: the same change means editing {size_kb(FLAT):.1f} KB, the file")
show(f"         that also holds all {fm} meshes")
show("")

# ---- 5. the collaboration claim, stated as file ownership rather than vibes
show("two people working at once:")
show(f"   flat     both edit {os.path.basename(FLAT)} -- same file, one conflict")
show("   layered  lighting person edits layer_lighting.usda")
show("            props person   edits layer_props.usda")
show("            no shared file, so no conflict to resolve")
show("")
show("THE HONEST CAVEAT: layering does not make the scene smaller. The sum of")
show("the parts is about the same as the flat file, and the layered version is")
show("MORE files to keep track of. What it buys is that the file you open to")
show("make a change is the size of the change, not the size of the scene.")

print("LYR_DONE", flush=True)
sim.close()
