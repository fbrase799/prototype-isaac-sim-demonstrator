# Pure rclpy publisher inside Isaac's python (full env), NO bridge involved.
# If the host sees this, DDS crosses the boundary and the bridge is the issue.
# If the host does not, the container's DDS cannot reach the host at all.
import sys
sys.path.insert(0, "/isaac-sim/exts/isaacsim.ros2.core/jazzy/rclpy")
import rclpy, time
from std_msgs.msg import String
rclpy.init()
n = rclpy.create_node("pure_pub")
p = n.create_publisher(String, "/pure_test", 10)
m = String(); m.data = "hello"
print("PURE> publishing /pure_test for 60s", flush=True)
t0 = time.time()
while time.time() - t0 < 60:
    p.publish(m); rclpy.spin_once(n, timeout_sec=0.01); time.sleep(0.05)
rclpy.shutdown()
