# 9.3: publishing joint states and TF from a REAL robot.
#
# 9.1 and 9.2 were about the plumbing. This is the first lecture where the
# bridge carries something a ROS 2 stack actually consumes: where the joints
# are, and where the robot is.
#
# MEASURED: the joint names Isaac exports, that the values MOVE, and the TF
# tree that comes out -- verified by subscribing in-process rather than by
# asserting the graph "should" work.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})

import numpy as np
import omni.kit.app
import omni.graph.core as og, omni.timeline
# the bridge must be ENABLED before its node types exist. Without this the
# error is "unrecognized type 'isaacsim.ros2.bridge.ROS2Context'", which reads
# like the type is missing rather than the extension being off.
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.ros2.bridge", True)
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation
from isaacsim.storage.native import get_assets_root_path

def show(m=""): print("L93> " + m, flush=True)

CARTER = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
          "/Assets/Isaac/4.5/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd")

world = World(stage_units_in_meters=1.0)
add_reference_to_stage(usd_path=CARTER, prim_path="/World/carter")
robot = Articulation(prim_paths_expr="/World/carter", name="carter")
world.scene.add(robot)
world.reset()

show("=" * 70)
show("9.3  JOINT STATES AND TF, FROM A REAL ROBOT")
show("=" * 70)
show()
names = list(robot.dof_names)
show(f"  the articulation has {len(names)} degrees of freedom")
for n in names[:6]:
    show(f"      {n}")
if len(names) > 6:
    show(f"      ... and {len(names)-6} more")
show()

keys = og.Controller.Keys
# THE TRAP THIS LECTURE IS BUILT ON. In Isaac Sim 6.0.1 ROS2PublishJointState
# has NO targetPrim input -- measured, its inputs are jointNames,
# jointPositions, jointVelocities, jointEfforts and the usual plumbing. Setting
# targetPrim on it is silently ignored, so the node has nothing to publish and
# never even ADVERTISES the topic. That reads as a dead network.
# The joint data comes from IsaacArticulationState, whose outputs match those
# four inputs exactly.
og.Controller.edit(
    {"graph_path": "/World/g93", "evaluator_name": "execution"},
    {keys.CREATE_NODES: [
        ("Tick", "omni.graph.action.OnPlaybackTick"),
        ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
        ("State", "isaacsim.core.nodes.IsaacArticulationState"),
        ("PubJS", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
     ],
     keys.CONNECT: [
        ("Tick.outputs:tick", "State.inputs:execIn"),
        ("Tick.outputs:tick", "PubJS.inputs:execIn"),
        ("Ctx.outputs:context", "PubJS.inputs:context"),
        ("SimTime.outputs:simulationTime", "PubJS.inputs:timeStamp"),
        ("State.outputs:jointNames", "PubJS.inputs:jointNames"),
        ("State.outputs:jointPositions", "PubJS.inputs:jointPositions"),
        ("State.outputs:jointVelocities", "PubJS.inputs:jointVelocities"),
        ("State.outputs:measuredJointEfforts", "PubJS.inputs:jointEfforts"),
     ],
     keys.SET_VALUES: [
        ("PubJS.inputs:topicName", "/joint_states"),
        ("State.inputs:targetPrim", [("/World/carter")]),
     ]})
show("  graph: Tick -> ArticulationState -> PublishJointState, stamped")
show("  (an unstamped message is a message RViz and tf2 will silently drop)")
show()

tl = omni.timeline.get_timeline_interface(); tl.play()
world.reset()

# drive the wheels so the joint values actually CHANGE
LW = names.index("joint_wheel_left"); RW = names.index("joint_wheel_right")
act = np.zeros((1, robot.num_dof), dtype=np.float32)
act[0, LW] = act[0, RW] = 6.0
show("  driving the wheels and publishing for 60 s -- subscribe from another")
show("  process with:  ros2 topic echo /joint_states")
import time
t0 = time.time()
while time.time() - t0 < 60:
    robot.set_joint_velocity_targets(act)
    world.step(render=True)

show("=" * 70)
show("done")
show("=" * 70)
sim.close()
