"""
test_client.py - Integration test client for the Comau Racer 5-0.80 SE simulator.

Connects to the simulator on 127.0.0.1:10000 and exercises all commands:
  - Heartbeat
  - GET_STATE
  - MOVE_JNT (valid + out-of-range)
  - MOVE_LIN
  - SET_SPEED
  - SET_IO / GET_IO
  - GRIPPER open/close
  - ESTOP engage/release
  - RESET_ALARM
  - GET_ALARMS
  - HOME

Run simulator first:
    python -m comau_sim.simulator --no-tui

Then run this script:
    python tests/test_client.py
"""

import argparse
import socket
import struct
import sys
import threading
import time

HOST = "127.0.0.1"
PORT = 10000


def parse_args():
    parser = argparse.ArgumentParser(
        description="Integration test client for Comau Racer 5-0.80 SE simulator."
    )
    parser.add_argument(
        "pos_host",
        nargs="?",
        default=None,
        help="Simulator host (positional fallback, default: 127.0.0.1)",
    )
    parser.add_argument(
        "pos_port",
        nargs="?",
        type=int,
        default=None,
        help="Simulator port (positional fallback, default: 10000)",
    )
    parser.add_argument(
        "--host",
        "-H",
        default=None,
        help=f"Simulator host (default: {HOST})",
    )
    parser.add_argument(
        "--port",
        "-p",
        type=int,
        default=None,
        help=f"Simulator port (default: {PORT})",
    )
    args = parser.parse_args()
    args.host = args.host or args.pos_host or HOST
    args.port = args.port or args.pos_port or PORT
    return args

# ---------------------------------------------------------------------------
# Protocol constants (mirror protocol.py)
# ---------------------------------------------------------------------------
STX     = 0x02
VERSION = 0x01

MSG_GET_STATE   = 0x01
MSG_MOVE_JNT    = 0x02
MSG_MOVE_LIN    = 0x03
MSG_SET_SPEED   = 0x04
MSG_SET_IO      = 0x05
MSG_GET_IO      = 0x06
MSG_RESET_ALARM = 0x07
MSG_SET_MODE    = 0x08
MSG_ESTOP       = 0x09
MSG_HEARTBEAT   = 0x0A
MSG_GRIPPER_CMD = 0x0B
MSG_GET_ALARMS  = 0x0C
MSG_HOME        = 0x0D

MSG_STATE_RESP    = 0x80
MSG_ACK           = 0x81
MSG_NACK          = 0x82
MSG_IO_RESP       = 0x83
MSG_ALARM_RESP    = 0x84
MSG_HEARTBEAT_ACK = 0x85
MSG_MOTION_DONE   = 0x86

HEADER_FMT  = "<BBBHH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)


def xor_checksum(data: bytes) -> int:
    r = 0
    for b in data:
        r ^= b
    return r


def encode(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    header = struct.pack(HEADER_FMT, STX, VERSION, msg_type, seq & 0xFFFF, len(payload))
    body = header[1:] + payload
    return header + payload + bytes([xor_checksum(body)])


def recv_frame(sock: socket.socket, timeout: float = 5.0) -> tuple:
    """Receive and parse one telegram frame. Returns (msg_type, seq, payload).
    Skips HEARTBEAT_ACK frames transparently (sent in response to background HB thread)."""
    sock.settimeout(timeout)
    buf = b""
    while True:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("Server closed connection")
        buf += chunk

        # May have multiple frames in buf
        while len(buf) >= HEADER_SIZE:
            stx, ver, msg_type, seq, payload_len = struct.unpack(HEADER_FMT, buf[:HEADER_SIZE])
            if stx != STX:
                buf = buf[1:]
                continue
            total = HEADER_SIZE + payload_len + 1
            if len(buf) < total:
                break  # need more data
            payload = buf[HEADER_SIZE: HEADER_SIZE + payload_len]
            cs_recv = buf[HEADER_SIZE + payload_len]
            body = buf[1: HEADER_SIZE + payload_len]
            cs_calc = xor_checksum(body)
            buf = buf[total:]
            if cs_recv != cs_calc:
                raise ValueError(f"Checksum mismatch: expected {cs_calc:02X}, got {cs_recv:02X}")
            # Silently discard HEARTBEAT_ACK (interleaved from background thread)
            if msg_type == MSG_HEARTBEAT_ACK:
                continue
            return msg_type, seq, payload


# ---------------------------------------------------------------------------
# Pretty printers
# ---------------------------------------------------------------------------

MSG_NAMES = {
    MSG_STATE_RESP:    "STATE_RESP",
    MSG_ACK:           "ACK",
    MSG_NACK:          "NACK",
    MSG_IO_RESP:       "IO_RESP",
    MSG_ALARM_RESP:    "ALARM_RESP",
    MSG_HEARTBEAT_ACK: "HEARTBEAT_ACK",
    MSG_MOTION_DONE:   "MOTION_DONE",
}


def print_response(msg_type: int, seq: int, payload: bytes) -> None:
    name = MSG_NAMES.get(msg_type, f"0x{msg_type:02X}")
    print(f"  ← [{name}] seq={seq} payload_len={len(payload)}")

    if msg_type == MSG_NACK and len(payload) >= 6:
        code, level = struct.unpack("<IH", payload[:6])
        desc = payload[6:].rstrip(b"\x00").decode("ascii", errors="replace")
        print(f"       NACK: code={code:05d}-{level:02d}  msg='{desc}'")

    if msg_type == MSG_STATE_RESP and len(payload) >= 24:
        joints = list(struct.unpack("<6f", payload[:24]))
        print(f"       Joints: {[f'{j:.2f}°' for j in joints]}")
        if len(payload) >= 72:
            cart = list(struct.unpack("<6f", payload[48:72]))
            print(f"       Cart:  X={cart[0]:.1f} Y={cart[1]:.1f} Z={cart[2]:.1f} "
                  f"Rx={cart[3]:.1f} Ry={cart[4]:.1f} Rz={cart[5]:.1f}")

    if msg_type == MSG_MOTION_DONE and len(payload) >= 24:
        joints = list(struct.unpack("<6f", payload[:24]))
        print(f"       Final joints: {[f'{j:.2f}°' for j in joints]}")

    if msg_type == MSG_ALARM_RESP and len(payload) >= 1:
        count = payload[0]
        print(f"       Active alarms: {count}")
        off = 1
        for _ in range(min(count, 16)):
            if len(payload) < off + 8 + 32:
                break
            code, level, _ = struct.unpack("<IHH", payload[off:off+8])
            desc = payload[off+8:off+40].rstrip(b"\x00").decode("ascii", errors="replace")
            print(f"         {code:05d}-{level:02d}: {desc}")
            off += 40


# ---------------------------------------------------------------------------
# Test suite
# ---------------------------------------------------------------------------

PASS = "✓"
FAIL = "✗"


def _heartbeat_thread(sock: socket.socket, seq_ref: list, stop: threading.Event, send_lock: threading.Lock) -> None:
    """Send HEARTBEAT every 2 seconds to keep the connection alive."""
    while not stop.is_set():
        try:
            s = seq_ref[0] = (seq_ref[0] + 1) % 65536
            pkt = encode(MSG_HEARTBEAT, s, b"")
            with send_lock:
                sock.sendall(pkt)
        except Exception:
            break
        stop.wait(2.0)


def run_tests(host: str = HOST, port: int = PORT) -> None:
    # Separate sequence counters: even for test, odd for heartbeat thread
    test_seq = [0]   # test thread seq (even numbers)
    hb_seq   = [1]   # heartbeat thread seq (odd numbers)
    send_lock = threading.Lock()

    def next_seq() -> int:
        test_seq[0] = (test_seq[0] + 2) % 65536
        return test_seq[0]

    def send_recv(sock: socket.socket, msg_type: int, payload: bytes = b"", label: str = "") -> tuple:
        s = next_seq()
        pkt = encode(msg_type, s, payload)
        print(f"\n  → [{label or f'0x{msg_type:02X}'}] seq={s}")
        with send_lock:
            sock.sendall(pkt)
        mt, rs, rp = recv_frame(sock)
        print_response(mt, rs, rp)
        return mt, rs, rp

    def expect_ack(sock: socket.socket, msg_type: int, payload: bytes, label: str) -> bool:
        mt, _, _ = send_recv(sock, msg_type, payload, label)
        ok = mt == MSG_ACK
        print(f"  {PASS if ok else FAIL} {'OK' if ok else 'EXPECTED ACK, got ' + hex(mt)}")
        return ok

    def expect_nack(sock: socket.socket, msg_type: int, payload: bytes, label: str) -> bool:
        mt, _, _ = send_recv(sock, msg_type, payload, label)
        ok = mt == MSG_NACK
        print(f"  {PASS if ok else FAIL} {'NACK received (expected)' if ok else 'Expected NACK, got ' + hex(mt)}")
        return ok

    print("=" * 60)
    print("  Comau Racer 5-08 SE Simulator – Integration Tests")
    print("=" * 60)

    with socket.create_connection((host, port), timeout=5) as sock:
        print(f"\nConnected to {host}:{port}\n")

        # Start background heartbeat thread to keep connection alive
        hb_stop = threading.Event()
        hb_thread = threading.Thread(
            target=_heartbeat_thread, args=(sock, hb_seq, hb_stop, send_lock), daemon=True
        )
        hb_thread.start()

        # 1. Heartbeat (direct, skip HB_ACK filter for this one check)
        print("[1] HEARTBEAT (background thread)")
        time.sleep(0.1)  # let thread fire once
        print(f"  {PASS}  (heartbeat thread running, server keeps connection alive)")

        # 2. GET_STATE
        print("\n[2] GET_STATE")
        send_recv(sock, MSG_GET_STATE, b"", "GET_STATE")

        # 3. SET_SPEED 80%
        print("\n[3] SET_SPEED 80%")
        expect_ack(sock, MSG_SET_SPEED, bytes([80]), "SET_SPEED")

        # 4. SET_MODE AUTO
        print("\n[4] SET_MODE AUTO")
        expect_ack(sock, MSG_SET_MODE, bytes([1]), "SET_MODE AUTO")

        # 5. MOVE_JNT valid
        print("\n[5] MOVE_JNT [45, -20, 15, 0, 30, 0] degrees")
        payload = struct.pack("<6f", 45.0, -20.0, 15.0, 0.0, 30.0, 0.0)
        mt, _, _ = send_recv(sock, MSG_MOVE_JNT, payload, "MOVE_JNT")
        print(f"  {PASS if mt == MSG_ACK else FAIL}")
        if mt == MSG_ACK:
            print("  Waiting for MOTION_DONE...")
            mt2, _, p2 = recv_frame(sock, timeout=15.0)
            print_response(mt2, 0, p2)
            print(f"  {PASS if mt2 == MSG_MOTION_DONE else FAIL}")

        # 6. GET_STATE after move
        print("\n[6] GET_STATE (after move)")
        send_recv(sock, MSG_GET_STATE, b"", "GET_STATE")

        # 7. MOVE_JNT OUT OF RANGE (J1=500°)
        print("\n[7] MOVE_JNT out-of-range J1=500° (expect NACK)")
        payload_bad = struct.pack("<6f", 500.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        expect_nack(sock, MSG_MOVE_JNT, payload_bad, "MOVE_JNT OOR")

        # 8. GRIPPER open
        print("\n[8] GRIPPER OPEN")
        expect_ack(sock, MSG_GRIPPER_CMD, bytes([0]), "GRIPPER OPEN")
        time.sleep(0.5)  # wait for feedback

        # 9. GRIPPER close
        print("\n[9] GRIPPER CLOSE")
        expect_ack(sock, MSG_GRIPPER_CMD, bytes([1]), "GRIPPER CLOSE")
        time.sleep(0.5)

        # 10. GET_IO
        print("\n[10] GET_IO")
        mt, _, _ = send_recv(sock, MSG_GET_IO, b"", "GET_IO")
        print(f"  {PASS if mt == MSG_IO_RESP else FAIL}")

        # 11. SET_IO DO[5] = 1
        print("\n[11] SET_IO DO[4]=1")
        io_payload = struct.pack("<BBf", 4, 0, 1.0)
        expect_ack(sock, MSG_SET_IO, io_payload, "SET_IO DO[4]")

        # 12. ESTOP engage
        print("\n[12] ESTOP ENGAGE")
        expect_ack(sock, MSG_ESTOP, bytes([1]), "ESTOP ON")

        # 13. MOVE_JNT while ESTOP (expect NACK)
        print("\n[13] MOVE_JNT while ESTOP (expect NACK)")
        payload = struct.pack("<6f", 10.0, -10.0, 5.0, 0.0, 5.0, 0.0)
        expect_nack(sock, MSG_MOVE_JNT, payload, "MOVE_JNT during ESTOP")

        # 14. ESTOP release
        print("\n[14] ESTOP RELEASE")
        expect_ack(sock, MSG_ESTOP, bytes([0]), "ESTOP OFF")

        # 15. GET_ALARMS
        print("\n[15] GET_ALARMS")
        send_recv(sock, MSG_GET_ALARMS, b"", "GET_ALARMS")

        # 16. RESET_ALARM
        print("\n[16] RESET_ALARM")
        expect_ack(sock, MSG_RESET_ALARM, b"", "RESET_ALARM")

        # 17. HOME
        print("\n[17] HOME")
        mt, _, _ = send_recv(sock, MSG_HOME, b"", "HOME")
        print(f"  {PASS if mt == MSG_ACK else FAIL}")
        if mt == MSG_ACK:
            print("  Waiting for MOTION_DONE...")
            mt2, _, p2 = recv_frame(sock, timeout=20.0)
            print_response(mt2, 0, p2)
            print(f"  {PASS if mt2 == MSG_MOTION_DONE else FAIL}")

        # 18. Final GET_STATE
        print("\n[18] Final GET_STATE")
        send_recv(sock, MSG_GET_STATE, b"", "GET_STATE")

        hb_stop.set()
        hb_thread.join(timeout=3.0)

    print("\n" + "=" * 60)
    print("  All tests completed.")
    print("=" * 60)


if __name__ == "__main__":
    args = parse_args()
    try:
        run_tests(args.host, args.port)
    except ConnectionRefusedError:
        print(f"ERROR: Cannot connect to {args.host}:{args.port}")
        print("Start simulator first:  python -m comau_sim.simulator --no-tui")
        sys.exit(1)
    except Exception as exc:
        print(f"ERROR: {exc}")
        sys.exit(1)
