# 9.4 camera: ROS2CameraHelper is the node that does the whole chain.
#
# ROS2PublishImage takes a raw GPU dataPtr, not a camera -- which is why wiring
# it by hand fails. ROS2CameraHelper takes a RENDER PRODUCT PATH and a type,
# and builds the reader/convert/publish chain internally.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import omni.kit.app, omni.graph.core as og, omni.timeline, omni.usd
import omni.replicator.core as rep
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import UsdGeom, Gf
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.ros2.bridge", True)
def show(m=""): print("CAM> " + str(m), flush=True)

W, H = 640, 360
world = World(stage_units_in_meters=1.0)
CARTER = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
          "/Assets/Isaac/4.5/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd")
add_reference_to_stage(usd_path=CARTER, prim_path="/World/carter")
world.reset()

stage = omni.usd.get_context().get_stage()
CAM = "/World/front_cam"
cam = UsdGeom.Camera.Define(stage, CAM)
cam.AddTranslateOp().Set(Gf.Vec3d(0.0, -3.0, 1.2))
rp = rep.create.render_product(CAM, (W, H))
rp_path = rp.path if hasattr(rp, "path") else str(rp)
show(f"render product: {rp_path}  ({W}x{H})")

k = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/World/gcam", "evaluator_name": "execution"},
    {k.CREATE_NODES: [("T","omni.graph.action.OnPlaybackTick"),
                      ("C","isaacsim.ros2.bridge.ROS2Context"),
                      ("H","isaacsim.ros2.bridge.ROS2CameraHelper")],
     k.CONNECT: [("T.outputs:tick","H.inputs:execIn"),
                 ("C.outputs:context","H.inputs:context")],
     k.SET_VALUES: [("H.inputs:renderProductPath", rp_path),
                    ("H.inputs:topicName","/rgb"),
                    ("H.inputs:type","rgb"),
                    ("H.inputs:frameId","front_cam"),
                    ("H.inputs:frameSkipCount", int(__import__("os").environ.get("SKIP","0")))]})
show("graph: Tick -> ROS2CameraHelper(type=rgb) on /rgb")

omni.timeline.get_timeline_interface().play(); world.reset()
show(f"publishing /rgb ({W}x{H}) skip={__import__("os").environ.get("SKIP","0")} for 70s")
import time; t0=time.time()
while time.time()-t0 < 70:
    world.step(render=True)
sim.close()
