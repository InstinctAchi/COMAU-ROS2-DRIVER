#!/usr/bin/env python3
"""Test: a TF target moves along +Y at constant speed and the robot follows it through MoveIt Servo.

Standalone (not part of the planner). Run it in the robot-planner container, with `run_servo.sh` already
running in another shell:

    python3 servo_follow_test.py --speed 0.03 --distance 0.30            # Servo in DRY mode: nothing moves
    python3 servo_follow_test.py --speed 0.03 --distance 0.30 --execute  # Servo started with --real

What it does
  1. reads the current pose of `tool0_tcp` in `world` (P0);
  2. publishes the TF `servo_test_target`: still at P0 for --hold-start s, then +Y at --speed up to
     --distance, then still for --hold-end s;
  3. follows that TF: reads it back from TF, feed-forward target speed + P on the position error,
     orientation held, sent as TwistStamped to Servo (--mode twist) or PoseStamped (--mode pose);
  4. logs a CSV and prints lag / error figures.

Safety: aborts (zero twist + pause Servo) if the TF is stale, if |error| > --max-err, if Servo reports a
halt, on Ctrl-C. Refuses to run against the real controller topic unless --execute is given.
"""
import argparse
import csv
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
from rcl_interfaces.srv import GetParameters
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from trajectory_msgs.msg import JointTrajectory

WORLD = "world"
TCP = "tool0_tcp"
TARGET = "servo_test_target"
REAL_TOPIC = "/joint_trajectory_controller/joint_trajectory"
# moveit_servo::StatusCode values that stop the motion
HALT_CODES = {-1: "INVALID", 2: "HALT_FOR_SINGULARITY", 5: "HALT_FOR_COLLISION", 6: "JOINT_BOUND"}
JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rotvec(q):
    """Rotation vector (rad) of a unit quaternion (shortest way)."""
    x, y, z, w = q
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    s = math.sqrt(x * x + y * y + z * z)
    if s < 1e-9:
        return (0.0, 0.0, 0.0)
    ang = 2.0 * math.atan2(s, w)
    return (x / s * ang, y / s * ang, z / s * ang)


def norm(v):
    return math.sqrt(sum(c * c for c in v))


def clamp_norm(v, limit):
    n = norm(v)
    if n > limit and n > 1e-12:
        k = limit / n
        return tuple(c * k for c in v)
    return tuple(v)


class FollowTest(Node):
    def __init__(self, args):
        super().__init__("servo_follow_test")
        self.a = args
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/servo_node/pose_target_cmds", 10)
        self.status = 0
        self.status_seen = False
        self.fly_speed = float("nan")
        self.joints = {}
        self.out_msgs = 0
        self.out_first = None
        self.out_last = None
        self.create_subscription(ServoStatus, "/servo_node/status", self._on_status, 10)
        self.create_subscription(Float32, "/comau/fly_lin_velocity_sent", self._on_fly, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 20)
        self.switch_cli = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        self.pause_cli = self.create_client(SetBool, "/servo_node/pause_servo")
        self.param_cli = self.create_client(GetParameters, "/servo_node/get_parameters")
        self.p0 = None
        self.q0 = None
        self.t0 = None
        self.rows = []
        self.aborted = None
        self.done = False
        self.motion_time = args.distance / args.speed
        self.total_time = args.hold_start + self.motion_time + args.hold_end

    # --- callbacks -------------------------------------------------------------------------
    def _on_status(self, msg):
        self.status = int(msg.code)
        self.status_seen = True

    def _on_fly(self, msg):
        self.fly_speed = float(msg.data)

    def _on_joints(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.joints[n] = p

    def _on_out(self, msg):
        self.out_msgs += 1
        if msg.points:
            pt = msg.points[0]
            rec = (self.get_clock().now().nanoseconds * 1e-9, list(msg.joint_names), list(pt.positions))
            if self.out_first is None:
                self.out_first = rec
            self.out_last = rec

    # --- helpers ---------------------------------------------------------------------------
    def spin_until(self, cond, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return True
        self.get_logger().error(f"timeout waiting for {what}")
        return False

    def lookup(self, child, when=None):
        tf = self.tf_buffer.lookup_transform(WORLD, child, Time() if when is None else when,
                                             timeout=Duration(seconds=0.0))
        t = tf.transform.translation
        r = tf.transform.rotation
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w), stamp

    def call(self, client, request, timeout=3.0):
        fut = client.call_async(request)
        end = time.monotonic() + timeout
        while time.monotonic() < end and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.02)
        return fut.result() if fut.done() else None

    def servo_output_topic(self):
        req = GetParameters.Request()
        req.names = ["moveit_servo.command_out_topic"]
        res = self.call(self.param_cli, req)
        if res is None or not res.values:
            return None
        return res.values[0].string_value

    def set_pause(self, paused):
        req = SetBool.Request()
        req.data = bool(paused)
        return self.call(self.pause_cli, req)

    def zero_twist(self, n=10):
        for _ in range(n):
            msg = TwistStamped()
            msg.header.frame_id = WORLD
            msg.header.stamp = self.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)

    # --- phases ----------------------------------------------------------------------------
    def prepare(self):
        a = self.a
        if not self.spin_until(lambda: self.tf_buffer.can_transform(WORLD, TCP, Time()), 8.0, f"TF {WORLD}->{TCP}"):
            return False
        # robot at rest for 0.5 s before taking P0
        last = None
        t_still = None
        for _ in range(60):
            rclpy.spin_once(self, timeout_sec=0.02)
            p, q, _ = self.lookup(TCP)
            if last is not None and norm([p[i] - last[i] for i in range(3)]) < 2e-4:
                t_still = t_still or time.monotonic()
                if time.monotonic() - t_still > 0.5:
                    break
            else:
                t_still = None
            last = p
        self.p0, self.q0, _ = self.lookup(TCP)
        self.get_logger().info(
            "P0 = (%.4f, %.4f, %.4f), q = (%.3f, %.3f, %.3f, %.3f)" % (*self.p0, *self.q0))
        for cli, name in ((self.switch_cli, "switch_command_type"), (self.pause_cli, "pause_servo"),
                          (self.param_cli, "get_parameters")):
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"/servo_node/{name} not available: is run_servo.sh running?")
                return False
        topic = self.servo_output_topic()
        self.out_topic = topic
        self.get_logger().info(f"Servo output topic: {topic}")
        if topic == REAL_TOPIC and not a.execute:
            self.get_logger().error("Servo writes to the REAL controller topic: add --execute to confirm")
            return False
        if topic != REAL_TOPIC and a.execute:
            self.get_logger().warn("--execute given but Servo is in DRY mode: the robot will not move")
        if topic:
            self.create_subscription(JointTrajectory, topic, self._on_out, 20)
        req = ServoCommandType.Request()
        req.command_type = 2 if a.mode == "pose" else 1
        res = self.call(self.switch_cli, req)
        if res is None or not res.success:
            self.get_logger().error("switch_command_type failed")
            return False
        if self.set_pause(False) is None:
            self.get_logger().error("pause_servo failed")
            return False
        return True

    def publish_target(self):
        t = self.get_clock().now()
        el = 0.0 if self.t0 is None else (time.monotonic() - self.t0)
        s = min(max(0.0, self.a.speed * (el - self.a.hold_start)), self.a.distance)
        msg = TransformStamped()
        msg.header.stamp = t.to_msg()
        msg.header.frame_id = WORLD
        msg.child_frame_id = TARGET
        msg.transform.translation.x = self.p0[0]
        msg.transform.translation.y = self.p0[1] + self.a.direction * s
        msg.transform.translation.z = self.p0[2]
        msg.transform.rotation.x, msg.transform.rotation.y = self.q0[0], self.q0[1]
        msg.transform.rotation.z, msg.transform.rotation.w = self.q0[2], self.q0[3]
        self.tf_pub.sendTransform(msg)

    def control_tick(self):
        a = self.a
        now = self.get_clock().now()
        el = time.monotonic() - self.t0
        try:
            tgt_p, tgt_q, tgt_stamp = self.lookup(TARGET)
            tcp_p, tcp_q, tcp_stamp = self.lookup(TCP)
        except Exception as exc:  # noqa: BLE001
            self.abort(f"TF lookup failed: {exc}")
            return
        age = now.nanoseconds * 1e-9 - tgt_stamp
        if age > 0.25:
            self.abort(f"TF target stale ({age:.2f} s)")
            return
        try:
            prev_p, _, prev_stamp = self.lookup(TARGET, Time(nanoseconds=int((tgt_stamp - 0.1) * 1e9)))
            dt = max(tgt_stamp - prev_stamp, 1e-3)
            v_ff = tuple((tgt_p[i] - prev_p[i]) / dt for i in range(3))
        except Exception:  # noqa: BLE001
            v_ff = (0.0, 0.0, 0.0)
        err = tuple(tgt_p[i] - tcp_p[i] for i in range(3))
        vmax = a.speed * 1.5 + 0.02
        lin = clamp_norm(tuple(v_ff[i] + a.kp * err[i] for i in range(3)), vmax)
        rerr = rotvec(quat_mul(tgt_q, quat_conj(tcp_q)))
        ang = clamp_norm(tuple(a.kp_rot * c for c in rerr), 0.3)

        if a.mode == "twist":
            msg = TwistStamped()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = WORLD
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = lin
            msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = ang
            self.twist_pub.publish(msg)
        else:
            msg = PoseStamped()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = WORLD
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = tgt_p
            (msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w) = tgt_q
            self.pose_pub.publish(msg)

        row = {"t": round(el, 4), "tgt_x": tgt_p[0], "tgt_y": tgt_p[1], "tgt_z": tgt_p[2],
               "tcp_x": tcp_p[0], "tcp_y": tcp_p[1], "tcp_z": tcp_p[2],
               "err_y": err[1], "err_norm": norm(err), "rot_err": norm(rerr),
               "cmd_vy": lin[1], "cmd_norm": norm(lin), "servo_status": self.status,
               "fly_lin_sent": self.fly_speed}
        for j in JOINTS:
            row[j] = self.joints.get(j, float("nan"))
        self.rows.append(row)

        if self.status in HALT_CODES:
            self.abort(f"Servo status {HALT_CODES[self.status]}")
        elif not a.no_abort and norm(err) > a.max_err:
            self.abort(f"error {norm(err) * 1000:.0f} mm > {a.max_err * 1000:.0f} mm")
        elif el >= self.total_time:
            self.done = True

    def abort(self, why):
        if self.aborted is None:
            self.aborted = why
            self.get_logger().error(f"ABORT: {why}")
        self.done = True

    def run(self):
        a = self.a
        if not self.prepare():
            return 2
        print(f"Start in {a.countdown:.0f} s: {'+' if a.direction > 0 else '-'}{a.distance * 1000:.0f} mm in Y at {a.speed * 1000:.0f} mm/s "
              f"({a.mode} mode, total {self.total_time:.1f} s). Ctrl-C to stop.")
        end = time.monotonic() + a.countdown
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        self.t0 = time.monotonic()
        next_tf = self.t0
        next_ctl = self.t0
        try:
            while rclpy.ok() and not self.done:
                now = time.monotonic()
                if now >= next_tf:
                    self.publish_target()
                    next_tf += 0.01
                if now >= next_ctl and self.tf_buffer.can_transform(WORLD, TARGET, Time()):
                    self.control_tick()
                    next_ctl += 1.0 / a.rate
                if now - next_ctl > 0.5:  # lost time: do not burst
                    next_ctl = now
                rclpy.spin_once(self, timeout_sec=0.002)
        except KeyboardInterrupt:
            self.aborted = self.aborted or "Ctrl-C"
        finally:
            self.zero_twist(12)
            self.set_pause(True)
        self.report()
        return 1 if self.aborted else 0

    def report(self):
        a = self.a
        os.makedirs(os.path.dirname(os.path.abspath(a.csv)) or ".", exist_ok=True)
        if self.rows:
            with open(a.csv, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
                w.writeheader()
                w.writerows(self.rows)
            print(f"CSV: {a.csv} ({len(self.rows)} rows)")
        print(f"Servo output messages seen on {getattr(self, 'out_topic', '?')}: {self.out_msgs}")
        if self.out_first and self.out_last and self.out_last[0] > self.out_first[0]:
            dq = [math.degrees(self.out_last[2][i] - self.out_first[2][i]) for i in range(len(self.out_last[2]))]
            rate = self.out_msgs / (self.out_last[0] - self.out_first[0])
            print("  rate %.0f Hz; joint change in the run (deg): %s" % (rate, ", ".join("%.2f" % d for d in dq)))
        if self.aborted:
            print(f"ABORTED: {self.aborted}")
        if len(self.rows) < 20:
            return
        t = [r["t"] for r in self.rows]
        tgt_y = [r["tgt_y"] for r in self.rows]
        tcp_y = [r["tcp_y"] for r in self.rows]
        m0 = a.hold_start + 1.5          # constant-speed window: after the start transient
        m1 = a.hold_start + self.motion_time
        idx = [i for i, x in enumerate(t) if m0 <= x <= m1]
        if len(idx) < 10:
            print("window too short for the statistics")
            return
        err = [tgt_y[i] - tcp_y[i] for i in idx]
        lat = [math.hypot(self.rows[i]["tgt_x"] - self.rows[i]["tcp_x"],
                          self.rows[i]["tgt_z"] - self.rows[i]["tcp_z"]) for i in idx]
        sp = (tcp_y[idx[-1]] - tcp_y[idx[0]]) / (t[idx[-1]] - t[idx[0]])
        mean = sum(err) / len(err)
        rms = math.sqrt(sum((e - mean) ** 2 for e in err) / len(err))
        # time lag: shift of the target that best matches the TCP
        best = None
        for lag_ms in range(0, 701, 10):
            lag = lag_ms / 1000.0
            sq, n = 0.0, 0
            for i in idx:
                tt = t[i] - lag
                j = max(0, min(len(t) - 1, int(round(tt / (t[-1] / (len(t) - 1))))))
                sq += (tcp_y[i] - tgt_y[j]) ** 2
                n += 1
            val = math.sqrt(sq / max(n, 1))
            if best is None or val < best[1]:
                best = (lag, val)
        print("--- constant-speed window %.1f..%.1f s -------------------------------" % (m0, m1))
        print("target speed      : %.1f mm/s" % (a.speed * 1000))
        print("TCP mean speed    : %.1f mm/s" % (sp * 1000))
        print("error Y (tgt-tcp) : mean %.1f mm, std %.1f mm, max |e| %.1f mm" % (mean * 1000, rms * 1000, max(abs(e) for e in err) * 1000))
        print("lateral error     : max %.1f mm (X/Z)" % (max(lat) * 1000))
        print("time lag          : %.0f ms (residual %.1f mm)" % (best[0] * 1000, best[1] * 1000))
        stat = sorted(set(r["servo_status"] for r in self.rows))
        print("servo status codes seen: %s" % stat)
        print("final error       : %.1f mm" % (self.rows[-1]["err_norm"] * 1000))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speed", type=float, default=0.03, help="m/s of the target (default 0.03)")
    ap.add_argument("--distance", type=float, default=0.30, help="m along Y, sign from --direction (default 0.30)")
    ap.add_argument("--direction", type=int, choices=[1, -1], default=1, help="+1 = +Y (default), -1 = -Y (to come back)")
    ap.add_argument("--hold-start", type=float, default=2.0)
    ap.add_argument("--hold-end", type=float, default=2.0)
    ap.add_argument("--mode", choices=["twist", "pose"], default="twist")
    ap.add_argument("--kp", type=float, default=1.5, help="1/s, position error gain (twist mode)")
    ap.add_argument("--kp-rot", type=float, default=1.0, help="1/s, orientation error gain")
    ap.add_argument("--rate", type=float, default=50.0, help="Hz of the control loop")
    ap.add_argument("--max-err", type=float, default=0.04, help="m: abort above this distance TCP-target")
    ap.add_argument("--no-abort", action="store_true", help="do not abort on the error (dry runs only)")
    ap.add_argument("--countdown", type=float, default=3.0)
    ap.add_argument("--execute", action="store_true", help="required when Servo writes to the real controller")
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()
    if args.csv is None:
        args.csv = "/tmp/servo_test_%s_%dmms.csv" % (time.strftime("%H%M%S"), round(args.speed * 1000))
    if args.speed <= 0 or args.distance <= 0 or args.distance > 0.5:
        print("speed > 0 and 0 < distance <= 0.5 m")
        return 2
    rclpy.init(signal_handler_options=rclpy.SignalHandlerOptions.NO)  # Ctrl-C = KeyboardInterrupt, context stays valid
    node = FollowTest(args)
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
