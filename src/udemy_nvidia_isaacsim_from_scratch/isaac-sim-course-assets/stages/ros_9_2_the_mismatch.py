# 9.2: "Enabling it, and the version mismatch that segfaults."
#
# The curriculum asked for this lecture from a 2026 forum thread. I did not
# have to go looking for the failure -- I hit every variant of it building this
# section, and this script reproduces the DIAGNOSIS rather than the crash.
#
# The crash itself cannot be shown in-process (a segfault takes the whole
# process down), so what is measured here is the STATE that distinguishes a
# working bridge from a broken one -- the four things to check, in order.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})

import os, ctypes
import omni.kit.app, carb
def show(m=""): print("L92> " + m, flush=True)

# SAMPLE FIRST. isaacsim/ros2/core/impl/extension.py line 75 does
# os.environ["ROS_DISTRO"] = ros_distro after it resolves the fallback, so
# reading this AFTER enabling always finds a value and inverts the diagnosis.
env_distro_before = os.environ.get("ROS_DISTRO")

mgr = omni.kit.app.get_app().get_extension_manager()
mgr.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)

show("=" * 70)
show("9.2  THE FOUR THINGS THAT DECIDE WHETHER THE BRIDGE WORKS")
show("=" * 70)
show()

# 1 -- which distro did it bind, and did it use its OWN libraries?
s = carb.settings.get_settings()
distro_set = s.get("/exts/isaacsim.ros2.bridge/ros_distro")
fallback = s.get("/exts/isaacsim.ros2.bridge/internal_lib_fallback")
env_distro = env_distro_before
show("1. WHICH ROS 2 IS IT USING?")
show(f"     ROS_DISTRO before load       : {env_distro!r}")
show(f"     ROS_DISTRO after load        : {os.environ.get('ROS_DISTRO')!r}"
     "   <- the bridge WRITES this")
show(f"     ros_distro setting           : {distro_set!r}")
show(f"     internal_lib_fallback        : {fallback}")
if env_distro is None and fallback:
    show("     -> using Isaac's OWN bundled libraries. This is what you want")
    show("        in a container with no sourced ROS 2.")
else:
    show("     -> expecting a USER-SOURCED ROS 2. If none exists, the bridge")
    show("        will find no libraries and publish nothing, silently.")
show()

# 2 -- do the backends actually load?
show("2. DO THE BACKEND LIBRARIES LOAD?")
base = "/isaac-sim/exts/isaacsim.ros2.core"
for d in ("humble", "jazzy"):
    p = f"{base}/bin/libisaacsim.ros2.core.{d}.so"
    try:
        ctypes.CDLL(p, mode=ctypes.RTLD_GLOBAL)
        n = len(os.listdir(f"{base}/{d}/lib"))
        show(f"     {d:7s} backend loads, {n} libraries bundled")
    except OSError as e:
        show(f"     {d:7s} FAILS: {str(e)[:80]}")
show()

# 3 -- does the core report a successful startup?
show("3. DOES THE CORE REPORT STARTUP OK?")
try:
    from isaacsim.ros2.core.bindings import _ros2_core
    i = _ros2_core.acquire_ros2_core_interface()
    st = i.get_startup_status()
    show(f"     get_startup_status() = {st}")
    show("     -> False here means the bridge disabled itself. Everything")
    show("        downstream will look like a network problem.")
except Exception as e:
    show(f"     could not query: {e}")
show()

# 4 -- the library path, which is what actually segfaults
show("4. LD_LIBRARY_PATH -- THE ONE THAT SEGFAULTS")
lp = os.environ.get("LD_LIBRARY_PATH", "")
first = lp.split(":")[0] if lp else "<unset>"
show(f"     first entry: {first}")
if "isaacsim.ros2.core" in first:
    show("     -> points at Isaac's own tree. Correct.")
else:
    show("     -> NOT Isaac's tree. Instantiating ROS2Context will segfault,")
    show("        and the node types will still register first, so the")
    show("        failure looks like 'it is there but refuses to work'.")
show()
show("     MEASURED WHILE BUILDING THIS SECTION: mounting a HOST ROS 2 at")
show("     /opt/ros and pointing LD_LIBRARY_PATH there does NOT help. Two")
show("     ROS 2 stacks resolving the same symbols crashes Kit on the")
show("     create call. The bundled tree is not a fallback -- it is the")
show("     supported path.")
show()
show("=" * 70)
show("done")
show("=" * 70)
sim.close()
