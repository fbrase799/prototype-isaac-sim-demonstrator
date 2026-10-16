# 4.3: where mass actually comes from, measured rather than asserted.
#
# The lecture is "worked" per the curriculum, so the arithmetic has to be real:
# every number the narration says is printed here by the running simulator, and
# the hand calculation is checked against what PhysX actually reports.
#
# What this answers:
#   1. What does a rigid body weigh if you set NOTHING? (density default)
#   2. What happens when you set density instead of mass?
#   3. What happens when you set BOTH? (which one wins)
#   4. Does mass come from the MESH or from the COLLIDER? (4.2's theme again)
#   5. Where does the centre of mass end up, and what moves it?
#
# Item 4 is the one that bites: a hollow shape approximated by a convex hull is
# weighed as the SOLID hull, so the mass Isaac computes is the mass of a shape
# you never modelled.
#
# SimulationApp FIRST, before any pxr import.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics, Gf
import omni.usd
from isaacsim.core.utils.stage import clear_stage
from isaacsim.core.api import World

WORLD = World(stage_units_in_meters=1.0)

# The test body: a 0.4 m cube. Chosen because its volume is trivial to check by
# hand on camera: 0.4^3 = 0.064 cubic metres.
SIZE = 0.4
VOLUME = SIZE ** 3


def fresh_stage():
    clear_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    w = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(w.GetPrim())
    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    return stage


def make_cube(stage, path, mass=None, density=None, z=1.0):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(SIZE)
    UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(0, 0, z))
    prim = c.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    m = UsdPhysics.MassAPI.Apply(prim)
    if mass is not None:
        m.CreateMassAttr(mass)
    if density is not None:
        m.CreateDensityAttr(density)
    return prim


def report(prim, world, steps=8):
    """Step the sim so PhysX resolves the sentinels, then read the values back.

    Reading straight after authoring returns the SENTINELS (mass 0.0, inertia
    zero, centre of mass -inf): those are the "not computed yet" markers from
    4.1, so a script that reads without stepping reports nothing useful.
    """
    world.reset()
    for _ in range(steps):
        world.step(render=False)
    import numpy as np
    from isaacsim.core.prims import RigidPrim
    rp = RigidPrim(str(prim.GetPath()))
    rp.initialize()
    mass = float(np.asarray(rp.get_masses())[0])
    # get_coms returns a TUPLE of (positions, orientations), not one array:
    # np.asarray on it raises "setting an array element with a sequence", which
    # is what made every case here report a TypeError.
    coms = rp.get_coms()
    com_pos = np.asarray(coms[0] if isinstance(coms, tuple) else coms)[0]
    # get_inertias is (N, 9), a flattened 3x3, so the diagonal is 0, 4 and 8.
    inertia = np.asarray(rp.get_inertias())[0]
    return (mass,
            [float(v) for v in np.ravel(com_pos)[:3]],
            [float(inertia[0]), float(inertia[4]), float(inertia[8])])


print("")
print("PHYSX_MASS")
print(f"a {SIZE} m cube, so volume = {SIZE}^3 = {VOLUME:.6f} m^3")
print("")
print(f"{'what was authored':<34} {'reported mass':>14} {'implied density':>16}")
print("-" * 70)

CASES = [
    ("nothing set at all", None, None),
    ("density = 1000 (water)", None, 1000.0),
    ("density = 2700 (aluminium)", None, 2700.0),
    ("mass = 5.0", 5.0, None),
    ("mass = 5.0 AND density = 2700", 5.0, 2700.0),
]

rows = []
for label, mass, density in CASES:
    stage = fresh_stage()
    prim = make_cube(stage, "/World/box", mass=mass, density=density)
    try:
        m, com, inertia = report(prim, WORLD)
        implied = m / VOLUME if VOLUME else 0.0
        print(f"{label:<34} {m:14.4f} {implied:16.1f}")
        rows.append((label, m, implied, com, inertia))
    except Exception as exc:                        # noqa: BLE001
        print(f"{label:<34} FAILED: {type(exc).__name__}: {str(exc)[:30]}")

# ---------------------------------------------------------------------------
# Part 2: is the mass computed from the MESH or from the COLLIDER?
#
# 4.2 showed a convex hull fills in a hollow. If mass is computed from the
# collider, then a hollow shape approximated by a hull is weighed as the SOLID
# hull: heavier than the thing you modelled, by exactly the volume of the gap.
print("")
print("PHYSX_MASS_SHAPE  (the same hollow channel, two approximations)")
print(f"{'approximation':<26} {'reported mass':>14} {'implied volume':>16}")
print("-" * 62)


def build_channel(stage, path):
    """Same U-channel as 4.2. Solid volume of the three slabs, computed by hand:
       base 1.6*1.0*0.10 = 0.160
       two walls 2 * 0.10*1.0*0.80 = 0.160
       total 0.320 m^3, against a bounding box of 1.6*1.0*0.90 = 1.440.
    """
    mesh = UsdGeom.Mesh.Define(stage, path)
    pts, counts, idx = [], [], []

    def box(cx, cy, cz, sx, sy, sz):
        base = len(pts)
        for dx in (-1, 1):
            for dy in (-1, 1):
                for dz in (-1, 1):
                    pts.append(Gf.Vec3f(cx + dx*sx/2, cy + dy*sy/2, cz + dz*sz/2))
        for f in [(0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3)]:
            counts.append(4)
            idx.extend([base+i for i in f])

    box(0.0, 0.0, 0.05, 1.6, 1.0, 0.10)
    box(-0.75, 0.0, 0.40, 0.10, 1.0, 0.80)
    box(0.75, 0.0, 0.40, 0.10, 1.0, 0.80)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(idx)
    return mesh


for approx in ("convexDecomposition", "convexHull"):
    stage = fresh_stage()
    m = build_channel(stage, "/World/channel")
    prim = m.GetPrim()
    UsdGeom.Xformable(m).AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.2))
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approx)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateDensityAttr(1000.0)
    try:
        mass, com, inertia = report(prim, WORLD)
        print(f"{approx:<26} {mass:14.3f} {mass/1000.0:16.4f}")
    except Exception as exc:                        # noqa: BLE001
        print(f"{approx:<26} FAILED: {type(exc).__name__}: {str(exc)[:26]}")

print("  hand check: the modelled solid is 0.320 m^3, its bounding box 1.440")

# ---------------------------------------------------------------------------
# Part 3: centre of mass, and what moves it.
print("")
print("PHYSX_MASS_COM")
print(f"{'what was authored':<34} {'centre of mass (m)':>30}")
print("-" * 68)
for label, com_set in (("nothing set: computed", None),
                       ("centerOfMass = (0, 0, 0.15)", Gf.Vec3f(0, 0, 0.15))):
    stage = fresh_stage()
    prim = make_cube(stage, "/World/box")
    mapi = UsdPhysics.MassAPI(prim)
    if com_set is not None:
        mapi.CreateCenterOfMassAttr(com_set)
    try:
        mass, com, inertia = report(prim, WORLD)
        print(f"{label:<34} {str([round(v,4) for v in com]):>30}")
    except Exception as exc:                        # noqa: BLE001
        print(f"{label:<34} FAILED: {type(exc).__name__}")

print("")
print("PHYSX_MASS_DONE", flush=True)
sim.close()
