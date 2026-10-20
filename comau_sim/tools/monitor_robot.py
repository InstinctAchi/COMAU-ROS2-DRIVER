#!/usr/bin/env python3
"""
monitor_robot.py - Terminal Dashboard (TUI) for Real Comau Robot Controller.

Connects to the physical Comau C5G controller (or simulator) and monitors:
  - Port 1104: State Server continuous 173-byte telemetry stream (50 Hz)
  - Port 1105: Robot Server connection & command status
  - Port 1101: Motion Server connection status
  - Port 21:   FTP Server reachability
  - Port 80:   HTTP Web Server reachability

Usage:
    python3 tools/monitor_robot.py [--host 192.168.1.10]
    ./tools/monitor_robot.py [HOST]
"""

import argparse
import curses
import datetime
import socket
import struct
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Default Configuration & Limits
# ---------------------------------------------------------------------------
DEFAULT_HOST = "192.168.1.10"
STATE_PORT = 1104
ROBOT_PORT = 1105
MOTION_PORT = 1101
FTP_PORT = 21
HTTP_PORT = 80

STATE_FMT = "<icIi10f6f6i6iII10i"
STATE_SIZE = struct.calcsize(STATE_FMT)  # 173 bytes

# Comau Racer 5-0.80 Joint Limits [deg]
JOINT_LIMITS = [
    (-170.0, 170.0),
    (-95.0, 135.0),
    (-155.0, 90.0),
    (-200.0, 200.0),
    (-100.0, 100.0),
    (-360.0, 360.0),
]

# Error Bitmask Definitions (from state_server.pdl / robot_server.pdl)
ERROR_DESCRIPTIONS = {
    0x00001: "Port 1104 Address In Use (KI_ERR_STATE_ACCEPT)",
    0x00002: "Port 1105 Address In Use (KI_ERR_ROBOT_ACCEPT)",
    0x00004: "Port 1101 Address In Use (KI_ERR_HANDLER_ACCEPT)",
    0x00008: "Port 1104 Write Error (KI_ERR_STATE_WRITE)",
    0x00010: "Robot Server Read NCON (KI_ERR_ROBOT_READ_NCON)",
    0x00020: "Motion Handler Read NCON (KI_ERR_HANDLER_READ_NCON)",
    0x00040: "Robot Server Disconnected (KI_ERR_ROBOT_READ_DCON)",
    0x00080: "Motion Handler Disconnected (KI_ERR_HANDLER_READ_DCON)",
    0x00100: "Robot Server Read Canceled (KI_ERR_ROBOT_READ_CANC)",
    0x00200: "Motion Handler Read Canceled (KI_ERR_HANDLER_READ_CANC)",
    0x00400: "Robot Server Read Timeout (KI_ERR_ROBOT_READ_TOUT)",
    0x00800: "Motion Handler Read Timeout (KI_ERR_HANDLER_READ_TOUT)",
    0x01000: "State Server Disconnected (KI_ERR_STATE_DISCONNECT)",
    0x02000: "Robot Server Disconnected (KI_ERR_ROBOT_DISCONNECT)",
    0x04000: "Motion Handler Disconnected (KI_ERR_HANDLER_DISCONNECT)",
    0x08000: "Safety Gate / Emergency Stop Open (KI_ERR_STATE_SAFETY_GATE)",
    0x10000: "Program Execution Error (KI_ERR_STATE_WRONG_MOTION)",
    0x20000: "System State in Alarm (KI_ERR_ROBOT_ALARM)",
    0x40000: "Motion Commanded with Drive Off (KI_ERR_MOTION_DRIVEOFF)",
}

ROBOT_STATUS_NAMES = {
    "R": ("READY", 1),       # Green
    "M": ("MOVING", 2),      # Cyan
    "P": ("PAUSED", 3),      # Yellow
    "I": ("RESUMING", 3),    # Yellow
    "S": ("SUCCEEDED", 1),   # Green
    "C": ("CANCELING", 3),   # Yellow
    "E": ("ERROR", 4),       # Red
    "T": ("TERMINATED", 4),  # Red
}


# ---------------------------------------------------------------------------
# Telemetry State Model
# ---------------------------------------------------------------------------
class RobotTelemetry:
    def __init__(self, host: str):
        self.host = host
        self.lock = threading.Lock()

        # Telemetry fields (from 1104)
        self.connected_1104 = False
        self.last_state_rx_time: Optional[float] = None
        self.timestamp = 0
        self.status = "?"
        self.sts_selector = 0
        self.sensor_type = 0
        self.joints: List[float] = [0.0] * 6
        self.ee: List[float] = [0.0] * 6
        self.din: List[int] = [0] * 6
        self.dout: List[int] = [0] * 6
        self.err_val = 0
        self.num_joints = 6

        # Stats
        self.packet_count = 0
        self.byte_count = 0
        self.rate_hz = 0.0
        self.conn_error_1104: Optional[str] = None

        # Ports probe status
        self.ports_status: Dict[int, Tuple[bool, float, str]] = {
            1104: (False, 0.0, "PENDING"),
            1105: (False, 0.0, "PENDING"),
            1101: (False, 0.0, "PENDING"),
            21:   (False, 0.0, "PENDING"),
            80:   (False, 0.0, "PENDING"),
        }

        # Status message
        self.user_message = "Initializing..."
        self.user_message_time = time.time()

    def set_message(self, msg: str):
        with self.lock:
            self.user_message = msg
            self.user_message_time = time.time()


# ---------------------------------------------------------------------------
# Background Workers
# ---------------------------------------------------------------------------
def _state_stream_worker(telemetry: RobotTelemetry, stop_event: threading.Event):
    """Continuously receive 173-byte state frames from port 1104."""
    sock: Optional[socket.socket] = None
    buf = bytearray()
    last_rate_calc = time.time()
    pkts_since_calc = 0

    while not stop_event.is_set():
        if sock is None:
            try:
                sock = socket.create_connection((telemetry.host, STATE_PORT), timeout=2.0)
                sock.settimeout(1.0)
                with telemetry.lock:
                    telemetry.connected_1104 = True
                    telemetry.conn_error_1104 = None
                    telemetry.ports_status[STATE_PORT] = (True, 0.1, "STREAMING")
                buf.clear()
            except Exception as e:
                with telemetry.lock:
                    telemetry.connected_1104 = False
                    telemetry.conn_error_1104 = str(e)
                    telemetry.ports_status[STATE_PORT] = (False, 0.0, "CONN REFUSED / TIMEOUT")
                sock = None
                stop_event.wait(1.0)
                continue

        try:
            chunk = sock.recv(4096)
            if not chunk:
                raise ConnectionError("Server closed state connection")

            buf.extend(chunk)
            now = time.time()

            while len(buf) >= STATE_SIZE:
                raw = bytes(buf[:STATE_SIZE])
                del buf[:STATE_SIZE]

                fields = struct.unpack(STATE_FMT, raw)
                ts, status_byte, sts_sel, sns_type = fields[0:4]
                jnts = fields[4:10]
                ee = fields[14:20]
                din = fields[20:26]
                dout = fields[26:32]
                err_val, num_j = fields[32:34]

                with telemetry.lock:
                    telemetry.timestamp = ts
                    telemetry.status = status_byte.decode("latin1", errors="replace")
                    telemetry.sts_selector = sts_sel
                    telemetry.sensor_type = sns_type
                    telemetry.joints = list(jnts)
                    telemetry.ee = list(ee)
                    telemetry.din = list(din)
                    telemetry.dout = list(dout)
                    telemetry.err_val = err_val
                    telemetry.num_joints = num_j
                    telemetry.packet_count += 1
                    telemetry.byte_count += STATE_SIZE
                    telemetry.last_state_rx_time = now

                pkts_since_calc += 1

            if now - last_rate_calc >= 1.0:
                elapsed = now - last_rate_calc
                with telemetry.lock:
                    telemetry.rate_hz = pkts_since_calc / elapsed
                pkts_since_calc = 0
                last_rate_calc = now

        except socket.timeout:
            # Keep alive
            continue
        except Exception as e:
            with telemetry.lock:
                telemetry.connected_1104 = False
                telemetry.conn_error_1104 = str(e)
                telemetry.ports_status[STATE_PORT] = (False, 0.0, "DISCONNECTED")
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
                sock = None
            stop_event.wait(0.5)

    if sock:
        try:
            sock.close()
        except Exception:
            pass


def _ports_probe_worker(telemetry: RobotTelemetry, stop_event: threading.Event):
    """Periodically probes ports 1105, 1101, 21, 80 to verify TCP reachability."""
    probed_ports = [ROBOT_PORT, MOTION_PORT, FTP_PORT, HTTP_PORT]

    while not stop_event.is_set():
        for port in probed_ports:
            t0 = time.time()
            ok = False
            rtt = 0.0
            desc = "OFFLINE"
            try:
                with socket.create_connection((telemetry.host, port), timeout=0.6) as s:
                    rtt = (time.time() - t0) * 1000.0
                    ok = True
                    desc = f"OPEN ({rtt:.1f}ms)"
            except ConnectionRefusedError:
                desc = "CONN REFUSED"
            except socket.timeout:
                desc = "TIMEOUT"
            except OSError as e:
                desc = f"UNREACHABLE ({e.strerror or e})"
            except Exception as e:
                desc = f"ERR {e}"

            with telemetry.lock:
                telemetry.ports_status[port] = (ok, rtt, desc)

            if stop_event.is_set():
                break

        stop_event.wait(1.5)


def send_robot_command(host: str, cmd_char: bytes) -> str:
    """Send single-character command ('R'=Reset, 'S'=Start) to Robot Server on port 1105."""
    try:
        with socket.create_connection((host, ROBOT_PORT), timeout=2.0) as s:
            msg_id = int(time.time()) % 100000
            pkt = struct.pack("<Ic", msg_id, cmd_char)
            s.sendall(pkt)
            return f"Command '{cmd_char.decode()}' sent to port 1105 (id: {msg_id})"
    except Exception as e:
        return f"Failed to send '{cmd_char.decode()}': {e}"


# ---------------------------------------------------------------------------
# Curses UI Dashboard
# ---------------------------------------------------------------------------
def _draw_tui(stdscr, telemetry: RobotTelemetry, stop_event: threading.Event):
    curses.curs_set(0)
    curses.start_color()
    curses.use_default_colors()

    # Color pairs
    curses.init_pair(1, curses.COLOR_GREEN, -1)   # OK / READY
    curses.init_pair(2, curses.COLOR_CYAN, -1)    # MOVING / INFO
    curses.init_pair(3, curses.COLOR_YELLOW, -1)  # WARNING / MANUAL
    curses.init_pair(4, curses.COLOR_RED, -1)     # ERROR / ALARM
    curses.init_pair(5, curses.COLOR_MAGENTA, -1) # REMOTE / SPECIAL

    stdscr.nodelay(True)
    stdscr.timeout(100)

    paused = False

    while not stop_event.is_set():
        if not paused:
            stdscr.erase()
            max_y, max_x = stdscr.getmaxyx()

            # Take thread-safe snapshot
            with telemetry.lock:
                conn_1104 = telemetry.connected_1104
                err_1104 = telemetry.conn_error_1104
                st = telemetry.status
                sel = telemetry.sts_selector
                joints = list(telemetry.joints)
                ee = list(telemetry.ee)
                din = list(telemetry.din)
                dout = list(telemetry.dout)
                err_val = telemetry.err_val
                rate = telemetry.rate_hz
                pkts = telemetry.packet_count
                bytes_rx = telemetry.byte_count
                last_rx = telemetry.last_state_rx_time
                ports = dict(telemetry.ports_status)
                msg = telemetry.user_message
                msg_time = telemetry.user_message_time

            def put(y: int, x: int, text: str, attr: int = curses.A_NORMAL):
                if 0 <= y < max_y and 0 <= x < max_x:
                    try:
                        stdscr.addstr(y, x, text[: max_x - x], attr)
                    except curses.error:
                        pass

            row = 0

            # 1. Header Box
            now_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            put(row, 0, "┌" + "─" * (max_x - 2) + "┐")
            row += 1
            title = f" COMAU RACER 5-0.80  │  C5G REAL CONTROLLER  │  TARGET: {telemetry.host}  │  {now_str} "
            put(row, 0, f"│{title.center(max_x - 2)}│", curses.A_BOLD)
            row += 1
            put(row, 0, "└" + "─" * (max_x - 2) + "┘")
            row += 1

            # 2. TCP/IP Ports Summary Line
            put(row, 2, "TCP/IP PORTS:", curses.A_BOLD)
            col = 16

            # 1104
            p1104_ok, _, p1104_desc = ports.get(STATE_PORT, (False, 0, "OFFLINE"))
            p1104_str = f"1104 [STATE]: {f'STREAMING {rate:.0f}Hz' if p1104_ok else p1104_desc}"
            p1104_attr = curses.color_pair(1) if p1104_ok else curses.color_pair(4)
            put(row, col, f"[{p1104_str}]", p1104_attr | curses.A_BOLD)
            col += len(p1104_str) + 4

            # 1105
            p1105_ok, _, p1105_desc = ports.get(ROBOT_PORT, (False, 0, "OFFLINE"))
            p1105_str = f"1105 [ROBOT]: {p1105_desc}"
            p1105_attr = curses.color_pair(1) if p1105_ok else curses.color_pair(3)
            put(row, col, f"[{p1105_str}]", p1105_attr)
            col += len(p1105_str) + 4

            # 1101
            p1101_ok, _, p1101_desc = ports.get(MOTION_PORT, (False, 0, "OFFLINE"))
            p1101_str = f"1101 [MOTION]: {p1101_desc}"
            p1101_attr = curses.color_pair(1) if p1101_ok else curses.color_pair(3)
            put(row, col, f"[{p1101_str}]", p1101_attr)
            col += len(p1101_str) + 4

            # 21 & 80
            p21_ok, _, _ = ports.get(FTP_PORT, (False, 0, ""))
            p80_ok, _, _ = ports.get(HTTP_PORT, (False, 0, ""))
            extra_str = f"FTP(21): {'UP' if p21_ok else 'DOWN'}  HTTP(80): {'UP' if p80_ok else 'DOWN'}"
            put(row, col, f"[{extra_str}]", curses.color_pair(1 if (p21_ok and p80_ok) else 3))
            row += 1

            put(row, 0, "─" * max_x)
            row += 1

            # 3. Controller State & Operating Mode
            st_name, st_color = ROBOT_STATUS_NAMES.get(st, (f"CODE: {st}", 3))
            put(row, 2, "ROBOT STATUS:", curses.A_BOLD)
            put(row, 16, f" {st_name} [{st}] ", curses.color_pair(st_color) | curses.A_BOLD | curses.A_REVERSE)

            # Operating mode decode
            # Bit 15 on -> T1(MANUAL), Bit 16 on -> AUTO, Bit 14 on -> REMOTE
            mode_str = "UNKNOWN"
            mode_color = 3
            if sel & 0x1:
                mode_str = "AUTO"
                mode_color = 2
            elif sel & 0x2:
                mode_str = "T1 (MANUAL)"
                mode_color = 3
            elif sel & 0x4:
                mode_str = "REMOTE"
                mode_color = 5

            drives_on = bool(sel & 0x8)
            start_btn = bool(sel & 0x10)
            safety_gate_err = bool(err_val & 0x08000)

            put(row, 36, "MODE:", curses.A_BOLD)
            put(row, 42, f" {mode_str} ", curses.color_pair(mode_color) | curses.A_BOLD)

            put(row, 56, "DRIVES:", curses.A_BOLD)
            drives_txt = " ON " if drives_on else " OFF "
            drives_col = curses.color_pair(1) if drives_on else curses.color_pair(4)
            put(row, 64, drives_txt, drives_col | curses.A_BOLD | curses.A_REVERSE)

            put(row, 73, "SAFETY:", curses.A_BOLD)
            safety_txt = " TRIPPED " if safety_gate_err else " OK "
            safety_col = curses.color_pair(4) if safety_gate_err else curses.color_pair(1)
            put(row, 81, safety_txt, safety_col | curses.A_BOLD)
            row += 1

            put(row, 0, "─" * max_x)
            row += 1

            # 4. Joint Positions & Cartesian Pose (Side by Side)
            split_x = max(46, max_x // 2)
            put(row, 2, "JOINT POSITIONS [ARM_JNTP]", curses.A_BOLD)
            put(row, split_x, "CARTESIAN TCP POSE [POS]", curses.A_BOLD)
            row += 1

            for i in range(6):
                lim_lo, lim_hi = JOINT_LIMITS[i]
                jval = joints[i]
                span = lim_hi - lim_lo
                pct = max(0.0, min(1.0, (jval - lim_lo) / span)) if span > 0 else 0.5
                bar_len = 10
                fill = int(pct * bar_len)
                bar = "█" * fill + "░" * (bar_len - fill)

                near_limit = jval < (lim_lo + 5.0) or jval > (lim_hi - 5.0)
                j_col = curses.color_pair(4) if near_limit else curses.color_pair(1)

                j_str = f"  J{i+1}: {jval:>8.2f}°  [{bar}] ({lim_lo:>4.0f}°..{lim_hi:>4.0f}°)"
                put(row, 2, j_str, j_col)

                # Cartesian TCP on right column
                cnames = ["X ", "Y ", "Z ", "Rx", "Ry", "Rz"]
                cunits = ["mm", "mm", "mm", "° ", "° ", "° "]
                cval = ee[i]
                c_str = f"  {cnames[i]}: {cval:>9.2f} {cunits[i]}"
                put(row, split_x, c_str, curses.color_pair(2))

                row += 1

            put(row, 0, "─" * max_x)
            row += 1

            # 5. Digital I/O
            put(row, 2, "DIGITAL I/O:", curses.A_BOLD)
            din_str = "DIN:  " + " ".join(f"[{i+1}:{din[i]}]" for i in range(6))
            dout_str = "DOUT: " + " ".join(f"[{i+1}:{dout[i]}]" for i in range(6))
            put(row, 16, din_str, curses.color_pair(2))
            put(row, split_x, dout_str, curses.color_pair(2))
            row += 1

            put(row, 0, "─" * max_x)
            row += 1

            # 6. Alarms & Error Bitmask Breakdown
            put(row, 2, f"CONTROLLER ERROR MASK: 0x{err_val:05X} ({err_val})", curses.A_BOLD)
            row += 1

            active_errors = []
            for mask, desc in ERROR_DESCRIPTIONS.items():
                if err_val & mask:
                    active_errors.append((mask, desc))

            if active_errors:
                for mask, desc in active_errors[:5]:
                    put(row, 4, f"• [0x{mask:05X}] {desc}", curses.color_pair(4) | curses.A_BOLD)
                    row += 1
            else:
                put(row, 4, "• No active errors / alarms reported by PDL server (KI_ERR_NULL)", curses.color_pair(1))
                row += 1

            put(row, 0, "─" * max_x)
            row += 1

            # 7. Telemetry Stream Stats
            rx_kb = bytes_rx / 1024.0
            rx_age = (time.time() - last_rx) if last_rx else 999.0
            stats_str = (
                f"  RX Packets: {pkts:<8} │ Total RX: {rx_kb:>7.1f} KB │ Rate: {rate:>4.1f} Hz │ "
                f"Age: {rx_age:.2f}s │ TS: {telemetry.timestamp}"
            )
            put(row, 0, stats_str, curses.A_NORMAL)
            row += 1

            # 8. User Action / Status Feedback
            if time.time() - msg_time < 5.0 and msg:
                put(row, 2, f"STATUS: {msg}", curses.color_pair(3) | curses.A_BOLD)
            row += 1

            # Footer / Keys
            put(max_y - 1, 0, " [Q] Quit  │  [C] Reset Alarm (Port 1105)  │  [S] Start ('S')  │  [Space] Pause ", curses.A_REVERSE)

            stdscr.refresh()

        # Keyboard input
        key = stdscr.getch()
        if key in (ord("q"), ord("Q"), 27):
            stop_event.set()
            break
        elif key in (ord(" "),):
            paused = not paused
        elif key in (ord("c"), ord("C")):
            telemetry.set_message("Sending Reset ('R') to port 1105...")
            res = send_robot_command(telemetry.host, b"R")
            telemetry.set_message(res)
        elif key in (ord("s"), ord("S")):
            telemetry.set_message("Sending Start ('S') to port 1105...")
            res = send_robot_command(telemetry.host, b"S")
            telemetry.set_message(res)


# ---------------------------------------------------------------------------
# Text / Snapshot Mode (for non-interactive shells or quick checks)
# ---------------------------------------------------------------------------
def print_text_snapshot(telemetry: RobotTelemetry):
    with telemetry.lock:
        st = telemetry.status
        sel = telemetry.sts_selector
        joints = list(telemetry.joints)
        ee = list(telemetry.ee)
        din = list(telemetry.din)
        dout = list(telemetry.dout)
        err_val = telemetry.err_val
        ports = dict(telemetry.ports_status)
        ts = telemetry.timestamp
        conn_1104 = telemetry.connected_1104
        err_1104 = telemetry.conn_error_1104

    now_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    st_name, _ = ROBOT_STATUS_NAMES.get(st, (f"CODE: {st}", 3))
    mode_str = "AUTO" if (sel & 0x1) else ("T1 (MANUAL)" if (sel & 0x2) else ("REMOTE" if (sel & 0x4) else "UNKNOWN"))
    drives_str = "ON" if (sel & 0x8) else "OFF"
    safety_str = "TRIPPED" if (err_val & 0x08000) else "OK"

    print("=" * 78)
    print(f"  COMAU RACER 5-0.80  │  C5G CONTROLLER SNAPSHOT  │  IP: {telemetry.host}")
    print(f"  Timestamp: {now_str} (Controller TS: {ts})")
    print("=" * 78)

    print("\n[TCP/IP PORTS STATUS]")
    for p in (1104, 1105, 1101, 21, 80):
        ok, rtt, desc = ports.get(p, (False, 0.0, "UNKNOWN"))
        pname = {1104: "State (1104)", 1105: "Robot (1105)", 1101: "Motion (1101)", 21: "FTP (21)", 80: "HTTP (80)"}.get(p, f"Port {p}")
        status_tag = "[OPEN]" if ok else "[CLOSED/ERROR]"
        extra = f"{rtt:.1f}ms" if ok and rtt > 0 else desc
        print(f"  • {pname:<16}: {status_tag:<14} {extra}")

    print("\n[CONTROLLER STATE]")
    print(f"  • Status : {st_name} ('{st}')")
    print(f"  • Mode   : {mode_str}")
    print(f"  • Drives : {drives_str}")
    print(f"  • Safety : {safety_str}")

    print("\n[JOINT POSITIONS]")
    for i in range(6):
        lim_lo, lim_hi = JOINT_LIMITS[i]
        jval = joints[i]
        span = lim_hi - lim_lo
        pct = max(0.0, min(1.0, (jval - lim_lo) / span)) if span > 0 else 0.5
        bar = "█" * int(pct * 10) + "░" * (10 - int(pct * 10))
        print(f"  • J{i+1}: {jval:>8.2f}°  [{bar}]  (limits: {lim_lo:>4.0f}° .. {lim_hi:>4.0f}°)")

    print("\n[CARTESIAN TCP]")
    print(f"  • Position (mm)   : X={ee[0]:.2f}, Y={ee[1]:.2f}, Z={ee[2]:.2f}")
    print(f"  • Orientation (°) : Rx={ee[3]:.2f}, Ry={ee[4]:.2f}, Rz={ee[5]:.2f}")

    print("\n[DIGITAL I/O]")
    print(f"  • DIN  : {' '.join(f'DI[{i+1}]={din[i]}' for i in range(6))}")
    print(f"  • DOUT : {' '.join(f'DO[{i+1}]={dout[i]}' for i in range(6))}")

    print(f"\n[ERROR MASK: 0x{err_val:05X}]")
    active_errors = [desc for mask, desc in ERROR_DESCRIPTIONS.items() if err_val & mask]
    if active_errors:
        for err in active_errors:
            print(f"  ⚠ {err}")
    else:
        print("  ✓ No active alarms/errors (KI_ERR_NULL)")
    print("=" * 78)


# ---------------------------------------------------------------------------
# CLI Argument Parser & Entry Point
# ---------------------------------------------------------------------------
def parse_args():
    parser = argparse.ArgumentParser(
        description="Terminal Dashboard (TUI) for Real Comau Robot Controller."
    )
    parser.add_argument(
        "pos_host",
        nargs="?",
        default=None,
        help=f"Robot Controller IP (positional fallback, default: {DEFAULT_HOST})",
    )
    parser.add_argument(
        "--host",
        "-H",
        default=None,
        help=f"Robot Controller IP (default: {DEFAULT_HOST})",
    )
    parser.add_argument(
        "--once",
        "--text",
        action="store_true",
        help="Print a single text snapshot and exit (non-interactive mode)",
    )
    args = parser.parse_args()
    args.host = args.host or args.pos_host or DEFAULT_HOST
    return args


def main():
    args = parse_args()
    telemetry = RobotTelemetry(args.host)
    stop_event = threading.Event()

    # Start background stream and probe workers
    t_stream = threading.Thread(target=_state_stream_worker, args=(telemetry, stop_event), daemon=True)
    t_probe = threading.Thread(target=_ports_probe_worker, args=(telemetry, stop_event), daemon=True)

    t_stream.start()
    t_probe.start()

    # If --once or non-interactive stdout, wait for first packet and print text snapshot
    if args.once or not sys.stdout.isatty():
        # Wait up to 2 seconds for initial telemetry
        t0 = time.time()
        while time.time() - t0 < 2.0:
            with telemetry.lock:
                if telemetry.packet_count > 0 and telemetry.ports_status[ROBOT_PORT][2] != "PENDING":
                    break
            time.sleep(0.05)
        stop_event.set()
        print_text_snapshot(telemetry)
        return

    # Interactive Curses Dashboard
    try:
        curses.wrapper(lambda scr: _draw_tui(scr, telemetry, stop_event))
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        t_stream.join(timeout=1.0)
        t_probe.join(timeout=1.0)


if __name__ == "__main__":
    main()
