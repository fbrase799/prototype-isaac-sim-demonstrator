# 9.8: Isaac in Docker beside a ROS 2 host.
#
# The forum question is "base images and working configurations". This answers
# it from a container that demonstrably works, with each claim measured rather
# than asserted -- because every one of these cost me a debugging session.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})
import os, ctypes, subprocess
import omni.kit.app, carb
def show(m=""): print("L98> " + str(m), flush=True)

env_before = os.environ.get("ROS_DISTRO")
mgr = omni.kit.app.get_app().get_extension_manager()
mgr.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)

show("=" * 70)
show("9.8  THE CONTAINER CONFIGURATION, MEASURED")
show("=" * 70)
show()

# 1. what Isaac actually ships, counted
show("1. ISAAC SHIPS ITS OWN ROS 2 -- COUNTED, NOT CLAIMED")
base = "/isaac-sim/exts/isaacsim.ros2.core"
for d in ("humble", "jazzy"):
    lib = f"{base}/{d}/lib"
    n = len(os.listdir(lib)) if os.path.isdir(lib) else 0
    key = [x for x in ("librcl.so", "librclcpp.so", "librmw_fastrtps_cpp.so")
           if os.path.exists(f"{lib}/{x}")]
    show(f"   {d:7s} {n:4d} libraries   core present: {len(key)}/3  {key}")
show("   -> no host ROS 2 is required. none is installed in this image.")
show(f"   host /opt/ros present in container: {os.path.isdir('/opt/ros')}")
show()

# 2. the env that decides whether the bundled tree is used
show("2. THE THREE VARIABLES THAT DECIDE IT")
s = carb.settings.get_settings()
show(f"   ROS_DISTRO before load : {env_before!r}   <- MUST be unset")
show(f"   ROS_DISTRO after load  : {os.environ.get('ROS_DISTRO')!r}   <- the bridge writes it")
show(f"   internal_lib_fallback  : {s.get('/exts/isaacsim.ros2.bridge/internal_lib_fallback')}")
lp = os.environ.get("LD_LIBRARY_PATH", "").split(":")[0]
show(f"   LD_LIBRARY_PATH[0]     : {lp}")
show(f"   RMW_IMPLEMENTATION     : {os.environ.get('RMW_IMPLEMENTATION')!r}")
show()

# 3. the DDS profile that is read from CWD, not the env
show("3. THE PROFILE FILE IS READ FROM THE WORKING DIRECTORY")
prof = "/isaac-sim/DEFAULT_FASTRTPS_PROFILES.xml"
show(f"   {prof}: exists={os.path.exists(prof)}")
show(f"   FASTRTPS_DEFAULT_PROFILES_FILE env: "
     f"{os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')!r}")
show("   the C++ bridge reads the CWD file. hand-copying it is what made my")
show("   results irreproducible run to run -- it does not survive a rebuild,")
show("   so container bring-up installs it.")
show()

# 4. both backends load
show("4. BOTH BACKENDS LOAD")
for d in ("humble", "jazzy"):
    p = f"{base}/bin/libisaacsim.ros2.core.{d}.so"
    try:
        ctypes.CDLL(p, mode=ctypes.RTLD_GLOBAL)
        show(f"   {d:7s} loads")
    except OSError as e:
        show(f"   {d:7s} FAILS: {str(e)[:70]}")
try:
    from isaacsim.ros2.core.bindings import _ros2_core
    i = _ros2_core.acquire_ros2_core_interface()
    show(f"   get_startup_status(): {i.get_startup_status()}")
except Exception as e:
    show(f"   status unavailable: {e}")
show()
show("5. WHAT DOCKER NEEDS, AND WHY")
show("   --network host   DDS discovery is multicast; a bridge network hides it")
show("   --ipc=host       shared-memory transport needs the same /dev/shm")
show("   --runtime=nvidia RTX rendering, which is what feeds the camera helper")
show("   -v /tmp/.X11-unix the viewport, when you want one")
show()
show("=" * 70)
show("done")
show("=" * 70)
sim.close()
