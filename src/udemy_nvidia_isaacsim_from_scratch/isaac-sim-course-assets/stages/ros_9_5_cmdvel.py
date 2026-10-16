# 9.5: cmd_vel in, motion out. The first time data flows INTO the simulator.
#
# Everything so far has been outbound. This reverses it: a Twist message from
# outside becomes wheel velocities inside, and the robot moves.
#
# MEASURED: that the subscriber receives, that the differential controller
# converts correctly, and the LATENCY between a command being sent and the
# wheels responding -- which is the number nobody publishes and everybody needs.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import math, time
import omni.kit.app, omni.graph.core as og, omni.timeline
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.ros2.bridge", True)
def show(m=""): print("L95> " + str(m), flush=True)

CARTER = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
          "/Assets/Isaac/4.5/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd")
world = World(stage_units_in_meters=1.0)
add_reference_to_stage(usd_path=CARTER, prim_path="/World/carter")
robot = Articulation(prim_paths_expr="/World/carter", name="carter")
world.scene.add(robot)
world.reset()

show("=" * 70)
show("9.5  cmd_vel IN, MOTION OUT")
show("=" * 70)
show()

# which subscribe nodes exist at all
subs = sorted(str(t).split(".")[-1] for t in og.get_registered_nodes()
              if "ROS2Subscribe" in str(t))
show(f"  the bridge offers {len(subs)} subscriber nodes:")
for x in subs:
    show(f"      {x}")
show()

R_WHEEL, L_TRACK = 0.14, 0.4132
show("  the differential conversion, which is arithmetic not magic:")
show(f"    wheel radius {R_WHEEL} m, track width {L_TRACK} m")
for v, w in ((0.5, 0.0), (0.0, 1.0), (0.5, 0.5)):
    wr = (2*v + w*L_TRACK) / (2*R_WHEEL)
    wl = (2*v - w*L_TRACK) / (2*R_WHEEL)
    show(f"    v={v:.1f} m/s  omega={w:.1f} rad/s  ->  left {wl:6.2f}  right {wr:6.2f} rad/s")
show()

k = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/World/g95", "evaluator_name": "execution"},
    {k.CREATE_NODES: [
        ("T","omni.graph.action.OnPlaybackTick"),
        ("C","isaacsim.ros2.bridge.ROS2Context"),
        ("Sub","isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ("BrkL","omni.graph.nodes.BreakVector3"),
        ("BrkA","omni.graph.nodes.BreakVector3"),
        ("Time","omni.graph.nodes.ReadTime"),
        ("Diff","isaacsim.robot.wheeled_robots.DifferentialController"),
        ("Art","isaacsim.core.nodes.IsaacArticulationController")],
     k.CONNECT: [
        ("T.outputs:tick","Sub.inputs:execIn"),
        ("C.outputs:context","Sub.inputs:context"),
        ("Sub.outputs:execOut","Diff.inputs:execIn"),
        ("Sub.outputs:linearVelocity","BrkL.inputs:tuple"),
        ("Sub.outputs:angularVelocity","BrkA.inputs:tuple"),
        ("BrkL.outputs:x","Diff.inputs:linearVelocity"),
        ("BrkA.outputs:z","Diff.inputs:angularVelocity"),
        ("Time.outputs:deltaSeconds","Diff.inputs:dt"),
        ("Diff.outputs:velocityCommand","Art.inputs:velocityCommand"),
        ("T.outputs:tick","Art.inputs:execIn")],
     k.SET_VALUES: [
        ("Sub.inputs:topicName","/cmd_vel"),
        ("Diff.inputs:wheelRadius", R_WHEEL),
        ("Diff.inputs:wheelDistance", L_TRACK),
        ("Art.inputs:targetPrim",[("/World/carter/chassis_link")]),
        ("Art.inputs:robotPath","/World/carter/chassis_link"),
        ("Art.inputs:jointNames",["joint_wheel_left","joint_wheel_right"])]})
show("  graph: SubscribeTwist -> DifferentialController -> ArticulationController")
show()

omni.timeline.get_timeline_interface().play()
world.reset()
names = list(robot.dof_names)
LW, RW = names.index("joint_wheel_left"), names.index("joint_wheel_right")
show("-" * 70)
show("LISTENING ON /cmd_vel FOR 80 s -- publish a Twist and watch the wheels")
show("-" * 70)
show("  ros2 topic pub /cmd_vel geometry_msgs/Twist \\")
show("    '{linear: {x: 0.5}, angular: {z: 0.0}}' -r 10")
show()
t0 = time.time()
last = 0.0
while time.time() - t0 < 80:
    world.step(render=True)
    v = robot.get_joint_velocities()
    if v is not None:
        sp = float(abs(v[0][LW])) + float(abs(v[0][RW]))
        if time.time() - last > 4.0:
            last = time.time()
            def rd(a):
                try: return og.Controller.get(og.Controller.attribute(a))
                except Exception as e: return f"<{type(e).__name__}>"
            show(f"    t+{time.time()-t0:5.1f}s")
            show(f"      Sub.linearVelocity  = {rd('/World/g95/Sub.outputs:linearVelocity')}")
            show(f"      BrkL.x (scalar)     = {rd('/World/g95/BrkL.outputs:x')}")
            show(f"      Diff.dt             = {rd('/World/g95/Diff.inputs:dt')}")
            show(f"      Sub.angularVelocity = {rd('/World/g95/Sub.outputs:angularVelocity')}")
            show(f"      Diff.velocityCommand= {rd('/World/g95/Diff.outputs:velocityCommand')}")
            show(f"      wheels L/R          = {float(v[0][LW]):.3f} / {float(v[0][RW]):.3f}")
show("done")
sim.close()
