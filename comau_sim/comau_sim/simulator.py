"""
simulator.py - Entry point for the Comau Racer 5-0.80 SE / R1C-6 TCP/IP simulator.

Launches:
  1. asyncio motion controller loop
  2. asyncio TCP server (port 10000)
  3. curses TUI dashboard (separate thread)

Usage:
    python -m comau_sim.simulator [--host HOST] [--port PORT] [--no-tui]
"""

import argparse
import asyncio
import curses
import logging
import sys
import threading
import time
from typing import List

from . import config
from .alarm_manager import AlarmManager
from .gripper import GripperController, GripperState
from .motion_controller import motion_loop
from .robot_state import RobotState, RobotStatus
from .tcp_server import run_server

log = logging.getLogger("comau")


# ---------------------------------------------------------------------------
# TUI Dashboard
# ---------------------------------------------------------------------------

_STATUS_COLOR = {
    RobotStatus.IDLE:   1,
    RobotStatus.MOVING: 2,
    RobotStatus.HOLD:   3,
    RobotStatus.ERROR:  4,
    RobotStatus.ESTOP:  4,
}

_MODE_COLOR = {
    "AUTO": 2,
    "MAN":  3,
    "PROG": 5,
}


def _run_tui(state: RobotState, alarm_mgr: AlarmManager, gripper: GripperController, stop_event: threading.Event) -> None:
    """Run curses TUI in a dedicated thread."""

    def draw(stdscr: "curses.window") -> None:
        curses.curs_set(0)
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN,  -1)  # IDLE / normal
        curses.init_pair(2, curses.COLOR_CYAN,   -1)  # MOVING / AUTO
        curses.init_pair(3, curses.COLOR_YELLOW, -1)  # HOLD / MAN / warning
        curses.init_pair(4, curses.COLOR_RED,    -1)  # ERROR / ESTOP / alarm
        curses.init_pair(5, curses.COLOR_MAGENTA,-1)  # PROG

        stdscr.nodelay(True)
        stdscr.timeout(100)

        while not stop_event.is_set():
            stdscr.erase()
            try:
                max_y, max_x = stdscr.getmaxyx()

                # Snapshot state (no lock held during draw)
                jp  = list(state.joint_pos)
                jv  = list(state.joint_vel)
                cp  = state.cartesian
                do_ = list(state.digital_out)
                di_ = list(state.digital_in)
                st  = state.status
                md  = state.mode.value
                drv = state.drives_on
                es  = state.estop_active
                ovr = state.speed_override
                cli = state.connected_client or "none"
                rxc = state.rx_count
                txc = state.tx_count
                alarms = alarm_mgr.format_tp_style()
                grip   = gripper.state

                sc = _STATUS_COLOR.get(st, 1)
                mc = _MODE_COLOR.get(md, 1)

                row = 0

                def put(y: int, x: int, text: str, attr: int = curses.A_NORMAL) -> None:
                    if 0 <= y < max_y and 0 <= x < max_x:
                        try:
                            stdscr.addstr(y, x, text[:max_x - x], attr)
                        except curses.error:
                            pass

                # Header
                header = f" COMAU {config.ROBOT_MODEL}  │  {config.CONTROLLER_MODEL} {config.CONTROLLER_RELEASE}  │  MODE: {md} "
                put(row, 0, "─" * max_x)
                row += 1
                put(row, 0, header.center(max_x), curses.color_pair(mc) | curses.A_BOLD)
                row += 1
                put(row, 0, "─" * max_x)
                row += 1

                # Joint + Cartesian side by side
                put(row, 0,  "  JOINT POSITIONS",          curses.A_BOLD)
                put(row, 42, "CARTESIAN TCP",              curses.A_BOLD)
                row += 1
                jnames = ["J1", "J2", "J3", "J4", "J5", "J6"]
                cnames = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
                cvals  = cp.to_list()
                cunits = ["mm","mm","mm","°","°","°"]
                for i in range(6):
                    lim_lo, lim_hi = config.JOINT_LIMITS_DEG[i]
                    pct = (jp[i] - lim_lo) / (lim_hi - lim_lo) if (lim_hi - lim_lo) > 0 else 0.5
                    bar_len = 10
                    bar = "█" * int(pct * bar_len) + "░" * (bar_len - int(pct * bar_len))
                    j_col = curses.color_pair(4) if (jp[i] < lim_lo * 0.95 or jp[i] > lim_hi * 0.95) else curses.color_pair(1)
                    put(row, 0,  f"  {jnames[i]}: {jp[i]:>8.2f}°  [{bar}]  vel:{jv[i]:>6.1f}°/s", j_col)
                    put(row, 42, f"  {cnames[i]:2}: {cvals[i]:>10.3f} {cunits[i]}", curses.color_pair(1))
                    row += 1

                put(row, 0, "─" * max_x); row += 1

                # Status bar
                status_str = st.value
                drives_str = "ON " if drv else "OFF"
                estop_str  = "ACTIVE" if es else "off"
                put(row, 0, f"  State: ", curses.A_BOLD)
                put(row, 9, f"{status_str:<8}", curses.color_pair(sc) | curses.A_BOLD)
                put(row, 18, f"Speed OVR: {ovr:3d}%   DRIVES: ", curses.A_NORMAL)
                drv_c = curses.color_pair(1) if drv else curses.color_pair(4)
                put(row, 48, drives_str, drv_c | curses.A_BOLD)
                put(row, 52, f"   E-STOP: ", curses.A_NORMAL)
                es_c = curses.color_pair(4) if es else curses.color_pair(1)
                put(row, 63, estop_str, es_c | curses.A_BOLD)
                row += 1

                put(row, 0, "─" * max_x); row += 1

                # I/O
                put(row, 0, "  DIGITAL OUTPUT", curses.A_BOLD)
                put(row, 42, "DIGITAL INPUT", curses.A_BOLD)
                row += 1
                do_str = " ".join(f"DO[{i+1}]={'1' if v else '0'}" for i, v in enumerate(do_[:8]))
                di_str = " ".join(f"DI[{i+1}]={'1' if v else '0'}" for i, v in enumerate(di_[:8]))
                put(row, 2, do_str[:38], curses.color_pair(1))
                put(row, 42, di_str[:38], curses.color_pair(1))
                row += 1
                grip_c = curses.color_pair(2) if grip == GripperState.OPEN else \
                         curses.color_pair(3) if grip == GripperState.MOVING else \
                         curses.color_pair(4) if grip == GripperState.FAULT else curses.color_pair(1)
                put(row, 2, f"GRIPPER (Camozzi CSPT): {grip}", grip_c | curses.A_BOLD)
                row += 1

                put(row, 0, "─" * max_x); row += 1

                # Alarms
                put(row, 0, "  ALARMS", curses.A_BOLD)
                row += 1
                if alarms:
                    for a in alarms[:4]:
                        put(row, 2, a, curses.color_pair(4))
                        row += 1
                else:
                    put(row, 2, "No active alarms", curses.color_pair(1))
                    row += 1

                put(row, 0, "─" * max_x); row += 1

                # Network
                put(row, 0, f"  TCP client: {cli}   RX: {rxc} msg   TX: {txc} msg", curses.A_NORMAL)
                row += 1
                put(row, 0, "─" * max_x)

            except curses.error:
                pass

            stdscr.refresh()

            # Handle quit key
            key = stdscr.getch()
            if key in (ord("q"), ord("Q"), 27):
                stop_event.set()
                break

    try:
        curses.wrapper(draw)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Comau Racer 5-0.80 SE / R1C-6 REL.2 TCP/IP Robot Simulator"
    )
    parser.add_argument("--host", default=config.TCP_HOST, help="Bind address")
    parser.add_argument("--port", type=int, default=config.TCP_PORT, help="TCP port (default: 10000)")
    parser.add_argument("--no-tui", action="store_true", help="Disable curses dashboard")
    parser.add_argument(
        "--ros2",
        action="store_true",
        default=True,
        help="Enable Comau ROS2 driver PDL servers (ports 1101, 1104, 1105; enabled by default)",
    )
    parser.add_argument(
        "--no-ros2",
        dest="ros2",
        action="store_false",
        help="Disable Comau ROS2 driver PDL servers",
    )
    parser.add_argument("--log-level", default="WARNING",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return parser.parse_args()


async def _async_main(args: argparse.Namespace) -> None:
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

    # Override network config from args
    config.TCP_HOST = args.host
    config.TCP_PORT = args.port

    state     = RobotState()
    alarm_mgr = AlarmManager()
    gripper   = GripperController(state, alarm_mgr)

    # Compute initial cartesian from home joints via FK
    from .kinematics import forward_kinematics
    _, pose = forward_kinematics(state.joint_pos)
    state.cartesian.x  = pose[0]
    state.cartesian.y  = pose[1]
    state.cartesian.z  = pose[2]
    state.cartesian.rx = pose[3]
    state.cartesian.ry = pose[4]
    state.cartesian.rz = pose[5]

    # Registry for motion_done callback (set by server, called by motion loop)
    motion_done_registry: list = []

    async def motion_done_cb(final_joints: List[float]) -> None:
        if motion_done_registry:
            await motion_done_registry[0](final_joints)

    # Launch tasks
    motion_task = asyncio.create_task(
        motion_loop(state, alarm_mgr, notify_done_cb=motion_done_cb),
        name="motion_loop",
    )
    server_task = asyncio.create_task(
        run_server(state, alarm_mgr, gripper, motion_done_registry),
        name="tcp_server",
    )

    try:
        await asyncio.gather(motion_task, server_task)
    except asyncio.CancelledError:
        motion_task.cancel()
        server_task.cancel()


def main() -> None:
    args = parse_args()
    stop_event = threading.Event()

    if not args.no_tui:
        # We need the event loop running before TUI starts (gripper init)
        state     = None  # will be set after loop start
        # TUI thread will be started after async setup

    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        logging.basicConfig(
            level=getattr(logging, args.log_level),
            format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        )

        config.TCP_HOST = args.host
        config.TCP_PORT = args.port

        state     = RobotState()
        alarm_mgr = AlarmManager()
        gripper   = GripperController(state, alarm_mgr)

        from .kinematics import forward_kinematics
        _, pose = forward_kinematics(state.joint_pos)
        state.cartesian.x  = pose[0]
        state.cartesian.y  = pose[1]
        state.cartesian.z  = pose[2]
        state.cartesian.rx = pose[3]
        state.cartesian.ry = pose[4]
        state.cartesian.rz = pose[5]

        motion_done_registry: list = []

        async def motion_done_cb(final_joints: List[float]) -> None:
            for cb in list(motion_done_registry):
                try:
                    await cb(final_joints)
                except Exception:
                    pass

        motion_task = loop.create_task(
            motion_loop(state, alarm_mgr, notify_done_cb=motion_done_cb),
            name="motion_loop",
        )
        server_task = loop.create_task(
            run_server(state, alarm_mgr, gripper, motion_done_registry),
            name="tcp_server",
        )

        ros2_bridge = None
        if args.ros2:
            from .ros2_pdl_servers import ComauRos2PdlBridge
            ros2_bridge = ComauRos2PdlBridge(state, alarm_mgr, motion_done_registry)
            loop.create_task(ros2_bridge.start(args.host), name="ros2_bridge_start")

        if not args.no_tui:
            tui_thread = threading.Thread(
                target=_run_tui,
                args=(state, alarm_mgr, gripper, stop_event),
                daemon=True,
            )
            tui_thread.start()
        else:
            print(f"Comau Racer 5-0.80 SE simulator running on {args.host}:{args.port}")
            if args.ros2:
                print(f"Comau ROS 2 PDL servers active on {args.host}: "
                      f"1101 (motion), 1104 (state), 1105 (robot)")
            print("Press Ctrl+C to stop.")

        def _on_stop() -> None:
            stop_event.set()
            motion_task.cancel()
            server_task.cancel()
            if ros2_bridge:
                loop.create_task(ros2_bridge.stop())

        loop.add_signal_handler(__import__("signal").SIGINT, _on_stop)
        loop.add_signal_handler(__import__("signal").SIGTERM, _on_stop)

        # Watch stop_event from the loop (TUI quit sets it)
        async def _stop_watcher() -> None:
            while not stop_event.is_set():
                await asyncio.sleep(0.2)
            _on_stop()

        loop.create_task(_stop_watcher())

        try:
            loop.run_until_complete(asyncio.gather(motion_task, server_task, return_exceptions=True))
        except (KeyboardInterrupt, asyncio.CancelledError):
            pass
        finally:
            loop.close()

    except Exception as exc:
        print(f"Fatal error: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
