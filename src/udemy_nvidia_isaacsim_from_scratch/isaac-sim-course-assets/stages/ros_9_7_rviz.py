# 9.7: RViz2 against the real stage.
#
# RViz is where every mistake from the last six lectures becomes visible at
# once -- and where a working topic can still show nothing, for reasons that
# have nothing to do with Isaac.
#
# MEASURED: what a TF tree needs to be ACCEPTED (not just published), the QoS
# each display expects, and the two frame-name rules that decide whether
# anything appears.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import time
import omni.kit.app, omni.graph.core as og, omni.timeline
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.ros2.bridge", True)
def show(m=""): print("L97> " + str(m), flush=True)

CARTER = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
          "/Assets/Isaac/4.5/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd")
world = World(stage_units_in_meters=1.0)
add_reference_to_stage(usd_path=CARTER, prim_path="/World/carter")
world.reset()

show("=" * 70)
show("9.7  RVIZ2 AGAINST THE REAL STAGE")
show("=" * 70)
show()

# publish a full tf tree + clock, which is the minimum RViz needs to draw
k = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/World/g97", "evaluator_name": "execution"},
    {k.CREATE_NODES: [
        ("T","omni.graph.action.OnPlaybackTick"),
        ("C","isaacsim.ros2.bridge.ROS2Context"),
        ("ST","isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("Clock","isaacsim.ros2.bridge.ROS2PublishClock"),
        ("TF","isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        ("Odom","isaacsim.ros2.bridge.ROS2PublishOdometry")],
     k.CONNECT: [
        ("T.outputs:tick","Clock.inputs:execIn"),
        ("C.outputs:context","Clock.inputs:context"),
        ("ST.outputs:simulationTime","Clock.inputs:timeStamp"),
        ("T.outputs:tick","TF.inputs:execIn"),
        ("C.outputs:context","TF.inputs:context"),
        ("ST.outputs:simulationTime","TF.inputs:timeStamp"),
        ("T.outputs:tick","Odom.inputs:execIn"),
        ("C.outputs:context","Odom.inputs:context"),
        ("ST.outputs:simulationTime","Odom.inputs:timeStamp")],
     k.SET_VALUES: [
        ("Clock.inputs:topicName","/clock"),
        ("TF.inputs:topicName","/tf"),
        ("TF.inputs:parentFrameId","odom"),
        ("TF.inputs:childFrameId","base_link"),
        ("Odom.inputs:topicName","/odom")]})
show("  publishing /clock, /tf (odom -> base_link) and /odom")
show()
show("  WHAT RVIZ NEEDS THAT A `ros2 topic echo` DOES NOT:")
show("    1. a FIXED FRAME that exists in the tree. Set it to a name nothing")
show("       publishes and every display goes grey with 'no transform'.")
show("    2. a tf tree with NO GAPS. Each display's frame must chain to the")
show("       fixed frame, and a single missing link greys the whole branch.")
show("    3. TIME that moves. Displays buffer by timestamp, so a frozen clock")
show("       looks exactly like a dead topic -- which is lecture one again.")
show("    4. QoS that MATCHES. Sensor data is usually BEST_EFFORT; RViz")
show("       defaults some displays to RELIABLE, and a mismatch subscribes")
show("       cleanly and delivers nothing.")
show()

# NO world.reset() after play(): resetting rewinds simulated time while the
# graph is publishing, and tf2 responds with "Detected jump back in time.
# Clearing TF buffer" on every single message.
omni.timeline.get_timeline_interface().play()
show("-" * 70)
show("HOLDING FOR 90 s -- run rviz2 on the host and add TF + Odometry")
show("-" * 70)
t0 = time.time()
while time.time() - t0 < 90:
    world.step(render=True)
sim.close()
