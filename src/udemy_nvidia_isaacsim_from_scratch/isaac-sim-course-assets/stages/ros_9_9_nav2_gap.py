# 9.9: what a Nav2 stack would still need from here.
#
# The honest handoff. This course has put real things on the wire across eight
# lectures. Nav2 needs a specific list. This measures the OVERLAP and the GAP
# rather than gesturing at "and then you run navigation".
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import omni.kit.app, omni.graph.core as og, omni.timeline
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.ros2.bridge", True)
def show(m=""): print("L99> " + str(m), flush=True)

CARTER = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
          "/Assets/Isaac/4.5/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd")
world = World(stage_units_in_meters=1.0)
add_reference_to_stage(usd_path=CARTER, prim_path="/World/carter")
world.reset()

show("=" * 70)
show("9.9  WHAT NAV2 WOULD STILL NEED")
show("=" * 70)
show()

# WHAT THIS COURSE HAS ALREADY PUT ON THE WIRE, with the lecture that did it
HAVE = [
    ("/clock",        "9.1", "sim time, and use_sim_time on every node"),
    ("/tf",           "9.3", "odom -> base_link, stamped"),
    ("/odom",         "9.3", "nav_msgs/Odometry"),
    ("camera topics", "9.4", "ROS2CameraHelper, 54.6 Hz at 640x360"),
    ("/cmd_vel",      "9.5", "Twist in, wheels at 3.571 rad/s"),
]
show("ALREADY ON THE WIRE, MEASURED IN THIS SECTION:")
for t, lec, note in HAVE:
    show(f"   {t:15s} {lec}   {note}")
show()

# WHAT NAV2 ADDITIONALLY REQUIRES
GAP = [
    ("map -> odom TF", "AMCL or SLAM publishes it. NOT the simulator's job."),
    ("/scan or a costmap-ready cloud",
     "ROS2PublishLaserScan / PointCloud exist; wiring an RTX lidar is the work"),
    ("a map, or SLAM", "nav2_map_server, or slam_toolbox building one live"),
    ("base_link -> sensor TFs", "one static transform per sensor frame"),
    ("/joint_states", "BROKEN in 6.0.1 -- measured in 9.3. Not needed by Nav2."),
    ("lifecycle + params", "nav2_bringup, and a params yaml per robot"),
]
show("WHAT NAV2 ADDITIONALLY NEEDS:")
for t, note in GAP:
    show(f"   {t:32s} {note}")
show()

# the honest scoring
show("-" * 70)
show("SCORING IT HONESTLY")
show("-" * 70)
show(f"   on the wire from this course : {len(HAVE)} of {len(HAVE)+len(GAP)}")
show(f"   still to do                  : {len(GAP)}")
show()
show("   the two that are genuinely the SIMULATOR's job are the lidar wiring")
show("   and the static sensor transforms. everything else is a ROS 2 task")
show("   that has nothing to do with Isaac, and pretending otherwise is how")
show("   a simulator course turns into a bad navigation course.")
show()
show("=" * 70)
show("done")
show("=" * 70)
sim.close()
