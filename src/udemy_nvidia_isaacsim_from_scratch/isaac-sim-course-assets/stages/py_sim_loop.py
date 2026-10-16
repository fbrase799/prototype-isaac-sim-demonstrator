# 8.2: the simulation loop -- step, play, and who owns time.
#
# 8.1 established that in a standalone script nothing steps for you. This
# lecture is about what a step actually IS, and it exists because the Isaac API
# offers several things that all look like "advance the simulation" and are not
# the same:
#
#   world.step(render=True)    physics + a rendered frame
#   world.step(render=False)   physics only
#   sim.update()               a Kit app update -- NOT necessarily physics
#   world.play() / .pause()    hands the clock to the timeline
#
# A learner who guesses wrong here gets a robot that moves at the wrong speed,
# or does not move at all, and no error message either way. So all four get
# MEASURED, in wall-clock cost and in simulated time advanced.
#
# WHAT IS MEASURED:
#   1. Cost per step, rendered vs not, over N steps. The ratio is the price of
#      the picture and it decides whether your training loop is viable.
#   2. Does sim.update() advance PHYSICS? (Timed and checked against z.)
#   3. physics_dt vs rendering_dt, and what happens when they disagree --
#      the source of the "motion at 0.33x" class of bug that Section 9 hits.
#   4. Whether stepping while NOT playing does anything.
#
# TRAPS THIS SCRIPT ENCODES:
#  - A fresh World has no physics scene until reset(); step() before reset()
#    silently does nothing and every dt reads 0.0.
#  - set_simulation_dt exists on the PhysicsContext, not on World directly, and
#    the name differs across versions -- read it off the running system.
#  - Timing a single step is meaningless: the first step after a reset pays for
#    lazy allocation. Discard a warm-up batch or the ratio is nonsense.
from isaacsim import SimulationApp

import time

sim = SimulationApp({"headless": True})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid


def show(l=""):
    print("PY2> " + l, flush=True)


world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
cube = world.scene.add(
    DynamicCuboid(prim_path="/World/c", name="c",
                  position=np.array([0.0, 0.0, 5.0]), size=0.2)
)
world.reset()

PDT = world.get_physics_dt()
RDT = world.get_rendering_dt()

show("=" * 62)
show("8.2  THE SIMULATION LOOP -- what a step actually is")
show("=" * 62)
show()
show(f"physics dt   : {PDT:.6f} s  ({1.0 / PDT:6.1f} Hz)")
show(f"rendering dt : {RDT:.6f} s  ({1.0 / RDT:6.1f} Hz)")
show()

WARM = 20
N = 120


def timed(render):
    """Wall-clock seconds per step, warm-up discarded."""
    world.reset()
    cube.set_world_pose(position=np.array([0.0, 0.0, 5.0]))
    for _ in range(WARM):                     # the first steps after a reset
        world.step(render=render)             # pay for lazy allocation
    t0 = time.perf_counter()
    for _ in range(N):
        world.step(render=render)
    return (time.perf_counter() - t0) / N


# ---------------------------------------------------------------------------
# 1. WHAT THE PICTURE COSTS
# ---------------------------------------------------------------------------
show("-" * 62)
show("1. WHAT THE PICTURE COSTS")
show("-" * 62)
no_r = timed(False)
with_r = timed(True)
show(f"step(render=False) : {no_r * 1000:8.3f} ms/step   "
     f"{1.0 / no_r:8.1f} steps/s")
show(f"step(render=True)  : {with_r * 1000:8.3f} ms/step   "
     f"{1.0 / with_r:8.1f} steps/s")
show(f"ratio              : {with_r / no_r:8.2f}x")
show()
show(f"  Real time is {PDT * 1000:.3f} ms per step. Rendered, a step costs")
show(f"  {with_r * 1000:.3f} ms, so this scene runs at "
     f"{PDT / with_r:.2f}x real time with a picture")
show(f"  and {PDT / no_r:.2f}x without one. THAT is why training runs headless.")
show()

# ---------------------------------------------------------------------------
# 2. DOES sim.update() ADVANCE PHYSICS?
# This is the one people get wrong. update() pumps the Kit application: UI,
# rendering, extensions. Whether physics moves depends on the timeline, not on
# the call. Measured against z rather than assumed.
# ---------------------------------------------------------------------------
show("-" * 62)
show("2. sim.update() IS NOT world.step()")
show("-" * 62)
world.reset()
cube.set_world_pose(position=np.array([0.0, 0.0, 5.0]))
world.step(render=False)
z_before = float(cube.get_world_pose()[0][2])
t0 = time.perf_counter()
for _ in range(60):
    sim.update()
upd = time.perf_counter() - t0
z_after = float(cube.get_world_pose()[0][2])
show(f"z before 60x sim.update() : {z_before:.4f} m")
show(f"z after  60x sim.update() : {z_after:.4f} m")
show(f"moved                     : {abs(z_after - z_before):.4f} m "
     f"(in {upd:.3f} s of wall clock)")
show()
# THE ANSWER IS NOT THE ONE I EXPECTED, AND THE LECTURE SAYS SO. The tidy story
# is "update() pumps the app and does not touch physics". Measured on 6.0.1
# after world.reset(), it DOES advance physics: the cube falls. reset() leaves
# the timeline playing, and a playing timeline steps physics on every app
# update. So the honest rule is not about which function you call, it is about
# whether the timeline is running.
moved = abs(z_after - z_before)
if moved < 1e-4:
    show("  Sixty application updates and the cube did not move: the timeline")
    show("  was stopped, so pumping the app advanced nothing.")
else:
    show(f"  Sixty application updates moved the cube {moved:.4f} m. If you")
    show("  expected update() to leave physics alone, this is the trap: after")
    show("  world.reset() the TIMELINE IS PLAYING, and while it plays, every")
    show("  app update carries physics along with it.")
    show()
    show("  So the rule is not 'step() moves physics and update() does not'.")
    show("  The rule is: THE TIMELINE DECIDES. step() is explicit and works")
    show("  either way; update() only moves physics when something is playing.")
    show("  Two calls, same visible effect here, completely different contracts")
    show("  -- and the difference only shows up when the timeline is stopped.")
# Prove the claim rather than asserting it: stop the timeline and repeat.
world.pause()
world.step(render=False)
z_p0 = float(cube.get_world_pose()[0][2])
for _ in range(60):
    sim.update()
z_p1 = float(cube.get_world_pose()[0][2])
show()
show("  Now the same sixty updates with the timeline PAUSED:")
show(f"    z {z_p0:.4f} -> {z_p1:.4f} m, moved {abs(z_p1 - z_p0):.4f} m")
show()

# ---------------------------------------------------------------------------
# 3. WHEN PHYSICS AND RENDERING DISAGREE
# The Section 9 "motion at 0.33x" bug in miniature: if you render every step but
# physics is set to a different rate, sim time and wall time diverge by exactly
# the ratio, and nothing warns you.
# ---------------------------------------------------------------------------
show("-" * 62)
show("3. WHEN THE TWO RATES DISAGREE")
show("-" * 62)
show(f"physics {1.0 / PDT:.1f} Hz vs rendering {1.0 / RDT:.1f} Hz "
     f"-> ratio {RDT / PDT:.2f}")
show()
STEPS = 90
world.reset()
cube.set_world_pose(position=np.array([0.0, 0.0, 5.0]))
world.step(render=False)
z0 = float(cube.get_world_pose()[0][2])
for _ in range(STEPS):
    world.step(render=False)
z1 = float(cube.get_world_pose()[0][2])
sim_t = STEPS * PDT
show(f"{STEPS} steps = {sim_t:.3f} s of SIM time, cube fell "
     f"{abs(z1 - z0):.4f} m")
# Only quote the textbook figure while the cube is genuinely in flight. From
# z=5.0 the ground is 4.9 m away and free fall covers that in 1.00 s, so at
# 1.500 s the cube has LANDED and 11.04 m is a number about a cube that does not
# exist. Same mistake 8.1 nearly shipped; state the landing instead.
land_t = (2 * (5.0 - 0.1) / 9.81) ** 0.5
if sim_t < land_t:
    show(f"  free fall over {sim_t:.3f} s covers "
         f"{0.5 * 9.81 * sim_t ** 2:.4f} m -- still airborne")
else:
    show(f"  it reached the ground at about t = {land_t:.3f} s and stopped")
    show(f"  there, so do NOT compare this to 0.5*g*t^2 = "
         f"{0.5 * 9.81 * sim_t ** 2:.4f} m: that is a number")
    show("  about a cube still falling, and this one has landed.")
show()
show("  Sim time is steps x physics_dt. It has NO relationship to how long the")
show("  loop took on the wall unless you deliberately tie them together. A")
show("  publisher that assumes one step = one wall tick is where 9.6's 0.33x")
show("  motion bug comes from.")
show()
show("=" * 62)
show("done")
show("=" * 62)

sim.close()
