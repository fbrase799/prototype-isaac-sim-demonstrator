# 9.1: what the ROS 2 bridge IS, measured rather than described.
#
# The claim this lecture has to earn: the bridge is an OmniGraph EXTENSION that
# turns simulator state into DDS traffic. Not a ROS node inside Isaac, not a
# network service, not a plugin you configure in a UI. A graph you build.
#
# MEASURED HERE:
#   1. how many node types the bridge registers, and what they are FOR
#   2. that a Context is a first-class graph node with a real handle
#   3. that publishing requires a TICK -- and which tick, which is the trap
#      that cost this course three days
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})

import omni.kit.app, omni.timeline
import omni.graph.core as og
from isaacsim.core.api import World

def show(m=""): print("L91> " + m, flush=True)

mgr = omni.kit.app.get_app().get_extension_manager()
mgr.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)

show("=" * 70)
show("9.1  WHAT THE ROS 2 BRIDGE ACTUALLY IS")
show("=" * 70)
show()

types = sorted(str(t) for t in og.get_registered_nodes() if "ROS2" in str(t))
show(f"  the bridge registers {len(types)} OmniGraph node types.")
show("  they fall into three jobs:")
pub = [t for t in types if "Publish" in t]
sub = [t for t in types if "Subscribe" in t]
oth = [t for t in types if t not in pub and t not in sub]
show(f"    PUBLISH  {len(pub):2d}  simulator state -> ROS 2 topics")
show(f"    SUBSCRIBE{len(sub):2d}  ROS 2 topics -> simulator commands")
show(f"    OTHER    {len(oth):2d}  context, clock, service, QoS plumbing")
show()
for t in pub[:6]:
    show(f"      {t.split('.')[-1]}")
show(f"      ... and {max(0,len(pub)-6)} more publishers")
show()

show("-" * 70)
show("A CONTEXT IS A GRAPH NODE, NOT A CONFIG FILE")
show("-" * 70)
world = World(stage_units_in_meters=1.0)
world.reset()
keys = og.Controller.Keys
(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/World/g91", "evaluator_name": "execution"},
    {keys.CREATE_NODES: [("Tick", "omni.graph.action.OnPlaybackTick"),
                         ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
                         ("Pub", "isaacsim.ros2.bridge.ROS2PublishClock")],
     keys.CONNECT: [("Tick.outputs:tick", "Pub.inputs:execIn"),
                    ("Ctx.outputs:context", "Pub.inputs:context")],
     keys.SET_VALUES: [("Pub.inputs:topicName", "/clock")]})
show(f"  built a 3-node graph: {[str(n).split('/')[-1].rstrip(chr(41)+chr(34)) for n in nodes]}")
omni.timeline.get_timeline_interface().play()
world.reset()
for _ in range(5):
    world.step(render=True)          # the graph must EVALUATE before the
                                     # context output carries a handle
ctx = og.Controller.get(og.Controller.attribute("/World/g91/Ctx.outputs:context"))
show(f"  after 5 ticks the context carries a handle: {ctx}")
show("  a DDS participant now exists. It is a value on a wire, not a setting.")
show()

show("-" * 70)
show("AND IT ONLY RUNS WHEN THE TIMELINE DOES")
show("-" * 70)
tl = omni.timeline.get_timeline_interface()
tl.play()
world.reset()
t0 = tl.get_current_time()
for _ in range(60):
    world.step(render=False)
t_norender = tl.get_current_time()
for _ in range(60):
    world.step(render=True)
t_render = tl.get_current_time()
show(f"  sim time at start           : {t0:.4f}")
show(f"  after 60 x step(render=False): {t_norender:.4f}   <- FROZEN")
show(f"  after 60 x step(render=True) : {t_render:.4f}   <- advancing")
show()
show("  OnPlaybackTick fires off the TIMELINE. render=False advances physics")
show("  and NOT the timeline, so the graph ticks once, publishes one message,")
show("  and every subscriber that attaches later sees an empty topic.")
show("  That failure is indistinguishable from a broken network.")
show()
show("=" * 70)
show("done")
show("=" * 70)
sim.close()
