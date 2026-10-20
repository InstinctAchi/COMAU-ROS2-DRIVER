import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from control_msgs.msg import JointTrajectoryControllerState
DUR = float(sys.argv[1])
class R(Node):
    def __init__(s):
        super().__init__("move_recorder")
        s.t0 = None
        s.create_subscription(JointTrajectoryControllerState, "/joint_trajectory_controller/controller_state", s.cb,
                              QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
    def cb(s, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if s.t0 is None: s.t0 = t
        ref = list(m.reference.positions); fb = list(m.feedback.positions)
        print("%.3f," % (t - s.t0) + ",".join("%.5f" % v for v in ref) + "," + ",".join("%.5f" % v for v in fb), flush=True)
rclpy.init(); n = R(); end = time.time() + DUR
while rclpy.ok() and time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)
