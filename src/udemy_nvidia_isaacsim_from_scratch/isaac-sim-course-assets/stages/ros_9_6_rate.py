# 9.6: "motion at 0.33x" -- the forum bug, tested rather than assumed.
#
# THE CLAIM: if the ROS 2 side samples at a different rate than the simulator
# steps, motion arrives slowed or sped by exactly that ratio, and it looks like
# a physics problem when it is a clock problem.
#
# MEASURED HERE: the physics dt, the render/publish tick rate, and the ratio
# between simulated time and wall time under different rate settings. If the
# 0.33x is real it will fall out of that ratio.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import time
import omni.kit.app, omni.graph.core as og, omni.timeline
from isaacsim.core.api import World
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.ros2.bridge", True)
def show(m=""): print("L96> " + str(m), flush=True)

world = World(stage_units_in_meters=1.0)
world.reset()
tl = omni.timeline.get_timeline_interface()

show("=" * 70)
show("9.6  WHEN SIMULATED TIME AND WALL TIME DISAGREE")
show("=" * 70)
show()
dt = world.get_physics_dt()
show(f"  physics dt          : {dt:.6f} s  ({1/dt:.1f} Hz)")
show()

# measure sim-time advance per second of WALL time, which is the ratio that
# decides whether motion looks right to a subscriber
show("-" * 70)
show("SIM SECONDS PER WALL SECOND, MEASURED")
show("-" * 70)
tl.play(); world.reset()
for label, nsteps in (("free-running", 0), ("throttled to 30 Hz", 30),
                      ("throttled to 20 Hz", 20), ("throttled to 10 Hz", 10)):
    t_sim0 = tl.get_current_time()
    t_w0 = time.time()
    n = 0
    while time.time() - t_w0 < 6.0:
        world.step(render=True)
        n += 1
        if nsteps:
            time.sleep(max(0.0, 1.0/nsteps - 0.0))
    t_sim1 = tl.get_current_time()
    t_w1 = time.time()
    sim_el = t_sim1 - t_sim0
    wall_el = t_w1 - t_w0
    ratio = sim_el / wall_el if wall_el else 0
    pred = (nsteps/(1/dt)) if nsteps else None
    ptxt = f"  predicted {pred:4.2f}x" if pred else ""
    show(f"  {label:20s} {n:4d} steps  sim {sim_el:6.3f} s / wall {wall_el:6.3f} s"
         f"  = {ratio:5.2f}x realtime{ptxt}")
show()
show("  a subscriber timestamps what it receives with SIM time. If sim time")
show("  advances slower than wall time, every downstream consumer -- rviz,")
show("  a controller, a bag file -- sees motion at exactly that ratio.")
show()
show("=" * 70)
show("done")
show("=" * 70)
sim.close()
