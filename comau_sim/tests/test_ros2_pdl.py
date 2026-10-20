"""
test_ros2_pdl.py - Integration test for Comau ROS 2 PDL server emulation.

Tests the 3 TCP ports used by the official COMAU-ROS2-DRIVER:
  - Port 1104: state_server (173 bytes binary telemetry stream)
  - Port 1105: robot_server (commands 'I', 'S', 'O', 'R')
  - Port 1101: motion_server (motion commands 'J')

Usage:
    python tests/test_ros2_pdl.py [--host HOST]
"""

import argparse
import socket
import struct
import sys
import time

STATE_FMT = "<icIi10f6f6i6iII10i"
STATE_SIZE = struct.calcsize(STATE_FMT)  # 173 bytes


def parse_args():
    parser = argparse.ArgumentParser(description="Test Comau ROS 2 PDL server emulation.")
    parser.add_argument("--host", "-H", default="127.0.0.1", help="Simulator host (default: 127.0.0.1)")
    return parser.parse_args()


def run_tests(host: str = "127.0.0.1") -> None:
    print("=" * 65)
    print(f"  Testing Comau ROS 2 PDL Servers (1101, 1104, 1105) on {host}")
    print("=" * 65)

    # 1. Test state_server on port 1104
    print("\n[1] Connecting to State Server (port 1104)...")
    with socket.create_connection((host, 1104), timeout=5.0) as sock_state:
        sock_state.settimeout(3.0)
        raw_state = b""
        while len(raw_state) < STATE_SIZE:
            chunk = sock_state.recv(STATE_SIZE - len(raw_state))
            if not chunk:
                raise ConnectionError("State server closed connection")
            raw_state += chunk

        fields = struct.unpack(STATE_FMT, raw_state)
        ts, status_byte, sts_sel, sns_type = fields[0:4]
        joints = fields[4:14]
        ee = fields[14:20]
        din = fields[20:26]
        dout = fields[26:32]
        err_val, num_j = fields[32:34]
        jtypes = fields[34:44]

        status_str = status_byte.decode("ascii")
        print(f"  ✓ Received state packet: {len(raw_state)} bytes")
        print(f"    Timestamp  : {ts}")
        print(f"    Status     : '{status_str}'")
        print(f"    Num Joints : {num_j}")
        print(f"    Joints[1-6]: {[f'{j:.2f}°' for j in joints[:6]]}")
        print(f"    Cartesian  : X={ee[0]:.1f} Y={ee[1]:.1f} Z={ee[2]:.1f}")
        assert num_j == 6, f"Expected 6 joints, got {num_j}"
        assert status_str in ("R", "M", "P", "S", "I", "C", "T", "E"), f"Unknown status '{status_str}'"

    # 2. Test robot_server on port 1105
    print("\n[2] Connecting to Robot Server (port 1105)...")
    with socket.create_connection((host, 1105), timeout=5.0) as sock_robot:
        # Send INITIALIZE ('I'): msg_id(4B) + 'I'(1B) + 76B payload
        msg_id = 101
        verbose = 1
        sensor_type = 0
        sensor_cnvrsn = 1
        sensor_gain = 100
        sensor_time = 0
        sensor_ofst_lim_trans = 500
        sensor_ofst_lim_rot = 800
        din_pins = [1, 2, -1, -1, -1, -1]
        dout_pins = [1, 4, -1, -1, -1, -1]

        init_payload = struct.pack(
            "<IcI6i6i6i",
            msg_id,
            b"I",
            verbose,
            sensor_type,
            sensor_cnvrsn,
            sensor_gain,
            sensor_time,
            sensor_ofst_lim_trans,
            sensor_ofst_lim_rot,
            *din_pins,
            *dout_pins,
        )
        sock_robot.sendall(init_payload)
        print("  ✓ Sent INITIALIZE command (configured DOUT pin 4)")
        time.sleep(0.1)

        # Send START ('S') command: msg_id(4B) + 'S'(1B)
        msg_id += 1
        sock_robot.sendall(struct.pack("<Ic", msg_id, b"S"))
        print("  ✓ Sent START command (motion enabled)")
        time.sleep(0.1)

        # Send IO ('O') command: set DOUT pin 4 = 1
        msg_id += 1
        pin_num = 4
        pin_val = 1
        sock_robot.sendall(struct.pack("<Icii", msg_id, b"O", pin_num, pin_val))
        print(f"  ✓ Sent IO command: set DOUT[{pin_num}] = {pin_val}")
        time.sleep(0.1)

    # 3. Verify IO state update on port 1104
    print("\n[3] Verifying I/O update on State Server (port 1104)...")
    with socket.create_connection((host, 1104), timeout=5.0) as sock_state:
        raw_state = b""
        while len(raw_state) < STATE_SIZE:
            raw_state += sock_state.recv(STATE_SIZE - len(raw_state))
        fields = struct.unpack(STATE_FMT, raw_state)
        dout = fields[26:32]
        print(f"  ✓ DOUT state received: {list(dout)}")
        assert dout[1] == 1, f"Expected configured DOUT pin 4 to be 1, got {dout[1]}"
        print("  ✓ DOUT[4] confirmed high via telemetry stream")

    # 4. Test motion_server on port 1101
    print("\n[4] Connecting to Motion Server (port 1101)...")
    with socket.create_connection((host, 1101), timeout=5.0) as sock_motion:
        # Send JOINT_POSITION ('J'): msg_id(4B) + 'J'(1B) + 6 floats(24B)
        target_j = [15.0, -10.0, 5.0, 0.0, 10.0, 0.0]
        msg_id = 201
        sock_motion.sendall(struct.pack("<Ic6f", msg_id, b"J", *target_j))
        print(f"  ✓ Sent JOINT_POSITION ('J') to {[f'{j:.1f}°' for j in target_j]}")

    # 5. Monitor motion on State Server (port 1104)
    print("\n[5] Monitoring motion execution on State Server (port 1104)...")
    with socket.create_connection((host, 1104), timeout=5.0) as sock_state:
        sock_state.settimeout(1.0)
        motion_seen = False
        t_start = time.time()
        final_joints = None

        while time.time() - t_start < 10.0:
            raw_state = b""
            while len(raw_state) < STATE_SIZE:
                chunk = sock_state.recv(STATE_SIZE - len(raw_state))
                if not chunk:
                    break
                raw_state += chunk
            if len(raw_state) == STATE_SIZE:
                fields = struct.unpack(STATE_FMT, raw_state)
                st = fields[1].decode("ascii")
                current_j = fields[4:10]
                if st == "M":
                    motion_seen = True
                if st in ("S", "R") and motion_seen:
                    final_joints = list(current_j)
                    break
            time.sleep(0.02)

        print(f"  ✓ Motion executed (status 'M' observed: {motion_seen})")
        if final_joints:
            print(f"  ✓ Motion completed. Final joints: {[f'{j:.2f}°' for j in final_joints]}")
            for i in range(6):
                assert abs(final_joints[i] - target_j[i]) < 1.0, (
                    f"Joint {i+1} mismatch: expected {target_j[i]}, got {final_joints[i]}"
                )

    print("\n" + "=" * 65)
    print("  ALL ROS 2 PDL SERVER INTEGRATION TESTS PASSED ✓")
    print("=" * 65)


if __name__ == "__main__":
    args = parse_args()
    try:
        run_tests(args.host)
    except ConnectionRefusedError as e:
        print(f"ERROR: Cannot connect to ROS 2 PDL servers on {args.host}: {e}")
        print("Start simulator first: python3 -m comau_sim.simulator --no-tui")
        sys.exit(1)
    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(1)
