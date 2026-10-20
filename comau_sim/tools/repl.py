#!/usr/bin/env python3
"""
Interactive REPL client for Comau Racer 5-08 SE simulator.

Usage:
    python3 tools/repl.py [--host HOST] [--port PORT]
    python3 tools/repl.py [host] [port]

Commands (type 'help' for list):
    state           -- read robot state
    io              -- read I/O
    alarms          -- list active alarms
    speed <0-100>   -- set speed override %
    mode <m|a|p>    -- set mode (manual/auto/program)
    jmove j1..j6    -- joint move [degrees]
    lmove x y z rx ry rz  -- linear move [mm / deg]
    home            -- move to home
    estop <on|off>  -- engage/release emergency stop
    do <ch> <0|1>   -- set digital output
    grip open|close -- gripper command
    reset           -- reset all alarms
    hb              -- send heartbeat
    quit / exit     -- disconnect
"""

import argparse
import socket
import struct
import sys
import threading
import time

# ---------------------------------------------------------------------------
# Low-level codec (self-contained, no imports from comau_sim)
# ---------------------------------------------------------------------------

STX          = 0x02
VERSION      = 0x01
HEADER_FMT   = "<BBBHH"
HEADER_SIZE  = struct.calcsize(HEADER_FMT)

MSG_HEARTBEAT    = 0x0A
MSG_GET_STATE    = 0x01
MSG_GET_IO       = 0x05
MSG_GET_ALARMS   = 0x07
MSG_SET_SPEED    = 0x04
MSG_SET_MODE     = 0x08
MSG_MOVE_JNT     = 0x02
MSG_MOVE_LIN     = 0x03
MSG_HOME         = 0x09
MSG_ESTOP        = 0x0C
MSG_SET_IO       = 0x06
MSG_GRIPPER_CMD  = 0x0B
MSG_RESET_ALARM  = 0x0D

MSG_STATE_RESP   = 0x80
MSG_ACK          = 0x81
MSG_NACK         = 0x82
MSG_IO_RESP      = 0x83
MSG_ALARM_RESP   = 0x84
MSG_HEARTBEAT_ACK= 0x85
MSG_MOTION_DONE  = 0x86

MSG_NAMES = {
    MSG_STATE_RESP:    "STATE_RESP",
    MSG_ACK:           "ACK",
    MSG_NACK:          "NACK",
    MSG_IO_RESP:       "IO_RESP",
    MSG_ALARM_RESP:    "ALARM_RESP",
    MSG_HEARTBEAT_ACK: "HEARTBEAT_ACK",
    MSG_MOTION_DONE:   "MOTION_DONE",
}


def _xor(data: bytes) -> int:
    r = 0
    for b in data:
        r ^= b
    return r


def encode(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    header = struct.pack(HEADER_FMT, STX, VERSION, msg_type, seq & 0xFFFF, len(payload))
    body = header[1:] + payload
    return header + payload + bytes([_xor(body)])


def recv_frame(sock: socket.socket, timeout: float = 10.0):
    """Receive one non-heartbeat-ack frame. Returns (msg_type, seq, payload) or None on error."""
    sock.settimeout(timeout)
    buf = b""
    while True:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            return None
        if not chunk:
            return None
        buf += chunk
        while len(buf) >= HEADER_SIZE:
            stx, ver, mt, sq, plen = struct.unpack(HEADER_FMT, buf[:HEADER_SIZE])
            if stx != STX:
                buf = buf[1:]
                continue
            total = HEADER_SIZE + plen + 1
            if len(buf) < total:
                break
            payload = buf[HEADER_SIZE: HEADER_SIZE + plen]
            cs_recv = buf[HEADER_SIZE + plen]
            body    = buf[1: HEADER_SIZE + plen]
            buf     = buf[total:]
            if _xor(body) != cs_recv:
                print(f"  [warn] checksum mismatch — frame discarded")
                continue
            if mt == MSG_HEARTBEAT_ACK:
                continue  # silently skip
            return mt, sq, payload


# ---------------------------------------------------------------------------
# Pretty printers
# ---------------------------------------------------------------------------

def print_state(payload: bytes) -> None:
    if len(payload) < 6 * 4 + 6 * 4 + 6 * 4:
        print("  [state] payload too short")
        return
    off = 0
    joints = list(struct.unpack_from("<6f", payload, off)); off += 24
    _jvel  = list(struct.unpack_from("<6f", payload, off)); off += 24
    cart   = list(struct.unpack_from("<6f", payload, off)); off += 24
    do_    = list(struct.unpack_from("<8B", payload, off)); off += 8
    di_    = list(struct.unpack_from("<8B", payload, off)); off += 8
    ao_    = list(struct.unpack_from("<4f", payload, off)); off += 16
    ai_    = list(struct.unpack_from("<4f", payload, off)); off += 16
    st, md, drv, es, ovr, na = struct.unpack_from("<BBBBBb", payload, off); off += 6
    status_map = {0:"IDLE",1:"MOVING",2:"HOLD",3:"ERROR",4:"ESTOP"}
    mode_map   = {0:"MANUAL",1:"AUTO",2:"PROG"}
    print(f"  Status : {status_map.get(st,'?')}  Mode: {mode_map.get(md,'?')}  "
          f"Drives: {'ON' if drv else 'OFF'}  EStop: {'YES' if es else 'NO'}  Speed: {ovr}%")
    print(f"  Joints : {[f'{j:.2f}°' for j in joints]}")
    print(f"  Cart   : X={cart[0]:.1f} Y={cart[1]:.1f} Z={cart[2]:.1f} "
          f"Rx={cart[3]:.1f} Ry={cart[4]:.1f} Rz={cart[5]:.1f}")
    do_str = " ".join(f"DO{i}={'1' if v else '0'}" for i, v in enumerate(do_))
    di_str = " ".join(f"DI{i}={'1' if v else '0'}" for i, v in enumerate(di_))
    print(f"  DOut   : {do_str}")
    print(f"  DIn    : {di_str}")
    print(f"  Alarms : {na}")


def print_io(payload: bytes) -> None:
    off = 0
    do_ = list(struct.unpack_from("<8B", payload, off)); off += 8
    di_ = list(struct.unpack_from("<8B", payload, off)); off += 8
    print(f"  DO: {do_}")
    print(f"  DI: {di_}")


def print_alarms(payload: bytes) -> None:
    if len(payload) < 2:
        print("  no alarm data")
        return
    count = struct.unpack_from("<H", payload, 0)[0]
    off = 2
    if count == 0:
        print("  no active alarms")
        return
    for _ in range(count):
        if off + 7 > len(payload):
            break
        code, level, desc_len = struct.unpack_from("<IHB", payload, off); off += 7
        desc = payload[off: off + desc_len].decode(errors="replace"); off += desc_len
        print(f"  {code:05d}-{level:02d}: {desc}")


def print_response(mt: int, seq: int, payload: bytes) -> None:
    name = MSG_NAMES.get(mt, f"0x{mt:02X}")
    print(f"  ← [{name}] seq={seq} payload={len(payload)}b")
    if mt == MSG_STATE_RESP:
        print_state(payload)
    elif mt == MSG_IO_RESP:
        print_io(payload)
    elif mt == MSG_ALARM_RESP:
        print_alarms(payload)
    elif mt == MSG_NACK and len(payload) >= 7:
        code  = struct.unpack_from("<I", payload, 0)[0]
        level = struct.unpack_from("<H", payload, 4)[0]
        slen  = payload[6]
        msg   = payload[7: 7 + slen].decode(errors="replace")
        print(f"  NACK {code:05d}-{level:02d}: {msg}")
    elif mt == MSG_MOTION_DONE and len(payload) >= 24:
        joints = list(struct.unpack_from("<6f", payload, 0))
        print(f"  Final joints: {[f'{j:.2f}°' for j in joints]}")


# ---------------------------------------------------------------------------
# Heartbeat background thread
# ---------------------------------------------------------------------------

def _hb_worker(sock: socket.socket, seq_ref: list, stop: threading.Event,
                send_lock: threading.Lock) -> None:
    while not stop.is_set():
        try:
            seq_ref[0] = (seq_ref[0] + 2) % 65536   # odd numbers for HB
            with send_lock:
                sock.sendall(encode(MSG_HEARTBEAT, seq_ref[0]))
        except Exception:
            break
        stop.wait(2.0)


# ---------------------------------------------------------------------------
# REPL
# ---------------------------------------------------------------------------

HELP = """
Commands:
  state                      read robot state
  io                         read digital/analog I/O
  alarms                     list active alarms
  speed <0-100>              set speed override %
  mode m|a|p                 set mode: m=manual a=auto p=program
  jmove <j1> .. <j6>        joint move [degrees]
  lmove <x> <y> <z> <rx> <ry> <rz>   Cartesian move [mm/deg]
  home                       move to home position
  estop on|off               engage/release e-stop
  do <channel> <0|1>         set digital output
  grip open|close            gripper command
  reset                      reset all alarms
  hb                         send heartbeat manually
  quit / exit                disconnect and exit
"""


def run_repl(host: str, port: int) -> None:
    hb_seq = [1]       # odd
    test_seq = [0]     # even
    send_lock = threading.Lock()

    def nseq() -> int:
        test_seq[0] = (test_seq[0] + 2) % 65536
        return test_seq[0]

    def send_cmd(mt: int, payload: bytes = b"") -> tuple:
        sq = nseq()
        pkt = encode(mt, sq, payload)
        with send_lock:
            sock.sendall(pkt)
        return sq

    def transact(mt: int, payload: bytes = b"", wait_motion: bool = False) -> None:
        sq = send_cmd(mt, payload)
        result = recv_frame(sock, timeout=8.0)
        if result is None:
            print("  [error] no response (timeout or connection closed)")
            return
        rmt, rsq, rp = result
        print_response(rmt, rsq, rp)
        if wait_motion and rmt == MSG_ACK:
            print("  [waiting for MOTION_DONE...]")
            r2 = recv_frame(sock, timeout=30.0)
            if r2:
                print_response(*r2)
            else:
                print("  [error] MOTION_DONE timeout")

    print(f"Connecting to {host}:{port}...")
    try:
        sock = socket.create_connection((host, port), timeout=5)
    except Exception as e:
        print(f"Cannot connect: {e}")
        sys.exit(1)

    print(f"Connected. Type 'help' for commands.\n")

    stop = threading.Event()
    hb_thread = threading.Thread(target=_hb_worker,
                                 args=(sock, hb_seq, stop, send_lock), daemon=True)
    hb_thread.start()

    try:
        while True:
            try:
                line = input("comau> ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not line:
                continue
            parts = line.split()
            cmd = parts[0].lower()

            if cmd in ("quit", "exit", "q"):
                break

            elif cmd == "help":
                print(HELP)

            elif cmd == "state":
                transact(MSG_GET_STATE)

            elif cmd == "io":
                transact(MSG_GET_IO)

            elif cmd == "alarms":
                transact(MSG_GET_ALARMS)

            elif cmd == "hb":
                transact(MSG_HEARTBEAT)

            elif cmd == "reset":
                transact(MSG_RESET_ALARM)

            elif cmd == "speed":
                if len(parts) != 2 or not parts[1].isdigit():
                    print("  usage: speed <0-100>"); continue
                transact(MSG_SET_SPEED, bytes([int(parts[1])]))

            elif cmd == "mode":
                if len(parts) != 2 or parts[1] not in ("m","a","p"):
                    print("  usage: mode m|a|p"); continue
                v = {"m": 0, "a": 1, "p": 2}[parts[1]]
                transact(MSG_SET_MODE, bytes([v]))

            elif cmd == "jmove":
                if len(parts) != 7:
                    print("  usage: jmove j1 j2 j3 j4 j5 j6 [degrees]"); continue
                try:
                    vals = [float(x) for x in parts[1:]]
                except ValueError:
                    print("  invalid numbers"); continue
                transact(MSG_MOVE_JNT, struct.pack("<6f", *vals), wait_motion=True)

            elif cmd == "lmove":
                if len(parts) != 7:
                    print("  usage: lmove x y z rx ry rz"); continue
                try:
                    vals = [float(x) for x in parts[1:]]
                except ValueError:
                    print("  invalid numbers"); continue
                transact(MSG_MOVE_LIN, struct.pack("<6f", *vals), wait_motion=True)

            elif cmd == "home":
                transact(MSG_HOME, wait_motion=True)

            elif cmd == "estop":
                if len(parts) != 2 or parts[1] not in ("on","off"):
                    print("  usage: estop on|off"); continue
                transact(MSG_ESTOP, bytes([1 if parts[1]=="on" else 0]))

            elif cmd == "do":
                if len(parts) != 3:
                    print("  usage: do <channel> <0|1>"); continue
                try:
                    ch = int(parts[1]); val = int(parts[2])
                except ValueError:
                    print("  invalid args"); continue
                # payload: channel(2B) io_type(1B=0=DO) value(1B)
                transact(MSG_SET_IO, struct.pack("<HBB", ch, 0, val))

            elif cmd == "grip":
                if len(parts) != 2 or parts[1] not in ("open","close"):
                    print("  usage: grip open|close"); continue
                v = 0 if parts[1] == "open" else 1
                transact(MSG_GRIPPER_CMD, bytes([v]))

            else:
                print(f"  unknown command '{cmd}' — type 'help'")

    finally:
        stop.set()
        hb_thread.join(timeout=3.0)
        sock.close()
        print("Disconnected.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Interactive REPL client for Comau Racer 5-08 SE simulator."
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
        help="Simulator host (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        "-p",
        type=int,
        default=None,
        help="Simulator port (default: 10000)",
    )
    args = parser.parse_args()
    host = args.host or args.pos_host or "127.0.0.1"
    port = args.port or args.pos_port or 10000
    return host, port


if __name__ == "__main__":
    host, port = parse_args()
    run_repl(host, port)
