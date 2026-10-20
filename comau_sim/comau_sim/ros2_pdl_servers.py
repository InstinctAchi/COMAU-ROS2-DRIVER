"""
ros2_pdl_servers.py - Multi-port TCP server emulation for official Comau ROS 2 driver.

Emulates the 3 PDL2 programs running on a Comau C5G/C5GPlus/R1C controller:
  1. state_server (port 1104): continuous 50 Hz binary telemetry stream (173 bytes)
  2. robot_server (port 1105): command execution and configuration ('I','S','R','X','L','U','O','T')
  3. motion_server (port 1101): trajectory and point-to-point motion commands ('J','T','F','C')

Matches:
  robots/models/Comau/COMAU-ROS2-DRIVER/comau_ros2_server/*.pdl
  robots/models/Comau/COMAU-ROS2-DRIVER/comau_ros2_client/comau_driver
"""

import asyncio
import logging
import math
import struct
import time
from typing import List, Optional

from . import config
from .alarm_manager import AlarmManager
from .kinematics import forward_kinematics
from .robot_state import RobotState, RobotStatus

log = logging.getLogger("comau.ros2_pdl")

# ---------------------------------------------------------------------------
# Binary Telegram Formats (matching comau_ros2_server PDL2 programs)
# ---------------------------------------------------------------------------

# state_message: 173 bytes
# timestamp(i), status(c), sts_selector(I), sensor_type(i),
# joint_position(10f), ee_position(6f), states_in(6i), states_out(6i),
# error_value(I), num_joints(I), jnt_type(10i)
STATE_MSG_FMT = "<icIi10f6f6i6iII10i"
STATE_MSG_SIZE = struct.calcsize(STATE_MSG_FMT)  # 173 bytes

# Command headers: id(uint32), command_type(char)
CMD_HEADER_FMT = "<Ic"
CMD_HEADER_SIZE = struct.calcsize(CMD_HEADER_FMT)  # 5 bytes


class ComauRos2PdlBridge:
    """
    Coordinates state_server, robot_server, and motion_server for the Comau ROS 2 driver.
    """

    def __init__(
        self,
        state: RobotState,
        alarm_mgr: AlarmManager,
        motion_done_registry: Optional[list] = None,
    ) -> None:
        self._state = state
        self._alarm = alarm_mgr
        self._motion_done_registry = motion_done_registry

        # PDL2 server state variables
        self.robot_status_char = b"R"  # 'R'=Ready, 'M'=Moving, 'P'=Paused, 'S'=Succeeded, 'E'=Error, 'T'=Terminated
        self.sensor_type = 0
        self.sensor_cnvrsn = 1
        self.sensor_gain = 100
        self.sensor_time = 0
        self.sensor_ofst_lim_trans = 500
        self.sensor_ofst_lim_rot = 800
        self.verbose = 1
        self.din_pins = [-1] * 6
        self.dout_pins = [-1] * 6
        self.enabled = False
        self.num_joints = 6

        # Active server tasks
        self._tasks: List[asyncio.Task] = []
        self._state_writers: List[asyncio.StreamWriter] = []

        # Motion queue
        self._motion_queue: asyncio.Queue = asyncio.Queue()
        self._trajectory_executor_task: Optional[asyncio.Task] = None

        # Register callback for motion completion if registry provided
        if self._motion_done_registry is not None:
            self._motion_done_registry.append(self._on_motion_controller_done)

    async def _on_motion_controller_done(self, final_joints: List[float]) -> None:
        """Called by motion_controller when a single motion completes."""
        if self.robot_status_char == b"M":
            self.robot_status_char = b"S"
            log.info("ROS 2 Motion completed. Status -> 'S' (Succeeded)")
            # Revert to Ready after brief delay
            await asyncio.sleep(0.1)
            if self.robot_status_char == b"S":
                self.robot_status_char = b"R"

    # -----------------------------------------------------------------------
    # State Server (Port 1104)
    # -----------------------------------------------------------------------

    def _pack_state_message(self) -> bytes:
        """Pack 173-byte state packet matching state_server.pdl."""
        now_ts = int(time.time())

        jp = list(self._state.joint_pos) + [0.0] * 4  # 10 values
        cp = self._state.cartesian.to_list()          # 6 values

        # Resolve configured DIN pins (-1 means bypass / default 0)
        din_vals = []
        for p in self.din_pins:
            if 1 <= p <= config.NUM_DIGITAL_IN:
                din_vals.append(1 if self._state.digital_in[p - 1] else 0)
            else:
                din_vals.append(0)

        # Resolve configured DOUT pins
        dout_vals = []
        for p in self.dout_pins:
            if 1 <= p <= config.NUM_DIGITAL_OUT:
                dout_vals.append(1 if self._state.digital_out[p - 1] else 0)
            else:
                dout_vals.append(0)

        error_val = 0
        if self._alarm.blocks_motion():
            error_val = 0x01000  # ERR_ALARM
        elif self._state.estop_active:
            error_val = 0x00400  # ERR_SAFETY_GATE

        jnt_types = [0] * 10  # 0 = revolute

        return struct.pack(
            STATE_MSG_FMT,
            now_ts,
            self.robot_status_char,
            0,                     # stsSelector
            self.sensor_type,      # sensor_type_feedback
            *jp,                   # joint_position[10]
            *cp,                   # ee_position[6]
            *din_vals,             # pins_state_in[6]
            *dout_vals,            # pins_state_out[6]
            error_val,             # error_value
            self.num_joints,       # num_joints
            *jnt_types,            # jnt_type[10]
        )

    async def _run_state_server(self, host: str, port: int) -> None:
        """Start TCP server on port 1104 and broadcast state at 50 Hz."""

        async def client_connected(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
            peer = writer.get_extra_info("peername")
            log.info("[StateServer:1104] Client connected: %s", peer)
            self._state_writers.append(writer)
            try:
                # Keep reading to detect disconnect
                while True:
                    data = await reader.read(1024)
                    if not data:
                        break
            except Exception:
                pass
            finally:
                if writer in self._state_writers:
                    self._state_writers.remove(writer)
                writer.close()
                log.info("[StateServer:1104] Client disconnected: %s", peer)

        server = await asyncio.start_server(client_connected, host=host, port=port)
        log.info("Comau ROS 2 State Server listening on %s:%d (50 Hz)", host, port)

        # Broadcast loop
        interval = 1.0 / config.ROS2_STATE_PUBLISH_HZ
        try:
            while True:
                t0 = time.monotonic()
                if self._state_writers:
                    packet = self._pack_state_message()
                    dead_writers = []
                    for w in list(self._state_writers):
                        try:
                            w.write(packet)
                        except Exception:
                            dead_writers.append(w)
                    for dw in dead_writers:
                        if dw in self._state_writers:
                            self._state_writers.remove(dw)
                elapsed = time.monotonic() - t0
                sleep_time = max(0.001, interval - elapsed)
                await asyncio.sleep(sleep_time)
        finally:
            server.close()
            await server.wait_closed()

    # -----------------------------------------------------------------------
    # Robot Server (Port 1105)
    # -----------------------------------------------------------------------

    async def _handle_robot_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        peer = writer.get_extra_info("peername")
        log.info("[RobotServer:1105] Client connected: %s", peer)

        buf = b""
        try:
            while True:
                chunk = await reader.read(4096)
                if not chunk:
                    break
                buf += chunk

                while len(buf) >= CMD_HEADER_SIZE:
                    msg_id, cmd_char = struct.unpack_from(CMD_HEADER_FMT, buf, 0)
                    cmd = cmd_char.decode("ascii", errors="replace")

                    # Handle commands
                    if cmd == "I":  # INITIALIZE
                        # 4B verbose + 24B sensors (6*4) + 24B din + 24B dout = 76B payload
                        needed = CMD_HEADER_SIZE + 76
                        if len(buf) < needed:
                            break
                        off = CMD_HEADER_SIZE
                        (
                            self.verbose,
                            self.sensor_type,
                            self.sensor_cnvrsn,
                            self.sensor_gain,
                            self.sensor_time,
                            self.sensor_ofst_lim_trans,
                            self.sensor_ofst_lim_rot,
                        ) = struct.unpack_from("<I6i", buf, off)
                        off += 28
                        self.din_pins = list(struct.unpack_from("<6i", buf, off))
                        off += 24
                        self.dout_pins = list(struct.unpack_from("<6i", buf, off))
                        off += 24
                        buf = buf[needed:]
                        self.robot_status_char = b"R"
                        log.info("[RobotServer] INITIALIZE: verbose=%d, din=%s, dout=%s",
                                 self.verbose, self.din_pins, self.dout_pins)

                    elif cmd == "S":  # START
                        buf = buf[CMD_HEADER_SIZE:]
                        self.enabled = True
                        self.robot_status_char = b"R"
                        log.info("[RobotServer] START: Robot motion enabled.")

                    elif cmd == "R":  # RESET
                        buf = buf[CMD_HEADER_SIZE:]
                        self._alarm.clear_all()
                        async with self._state.lock:
                            self._state.alarms.clear()
                            self._state.drives_on = True
                            self._state.status = RobotStatus.IDLE
                        self.robot_status_char = b"R"
                        log.info("[RobotServer] RESET: Robot reset complete, status -> 'R'.")

                    elif cmd == "X":  # CANCEL
                        buf = buf[CMD_HEADER_SIZE:]
                        self.robot_status_char = b"C"
                        async with self._state.lock:
                            self._state.motion_target_joints = None
                            self._state.status = RobotStatus.IDLE
                        await asyncio.sleep(0.05)
                        self.robot_status_char = b"R"
                        log.info("[RobotServer] CANCEL: Motion cancelled, status -> 'R'.")

                    elif cmd in ("L", "P"):  # LOCK / PAUSE
                        buf = buf[CMD_HEADER_SIZE:]
                        self.robot_status_char = b"P"
                        async with self._state.lock:
                            self._state.status = RobotStatus.HOLD
                        log.info("[RobotServer] LOCK/PAUSE: Robot paused, status -> 'P'.")

                    elif cmd == "U":  # UNLOCK
                        buf = buf[CMD_HEADER_SIZE:]
                        self.robot_status_char = b"R"
                        async with self._state.lock:
                            if self._state.status == RobotStatus.HOLD:
                                self._state.status = RobotStatus.IDLE
                        log.info("[RobotServer] UNLOCK: Robot unlocked, status -> 'R'.")

                    elif cmd == "O":  # IO
                        needed = CMD_HEADER_SIZE + 8  # pin(4B), state(4B)
                        if len(buf) < needed:
                            break
                        pin, pin_state = struct.unpack_from("<2i", buf, CMD_HEADER_SIZE)
                        buf = buf[needed:]
                        if 1 <= pin <= config.NUM_DIGITAL_OUT:
                            async with self._state.lock:
                                self._state.digital_out[pin - 1] = bool(pin_state)
                            log.info("[RobotServer] IO: Set DOUT[%d] = %d", pin, pin_state)

                    elif cmd == "C":  # CONFIGURATION
                        needed = CMD_HEADER_SIZE + 24
                        if len(buf) < needed:
                            break
                        (
                            self.sensor_type,
                            self.sensor_cnvrsn,
                            self.sensor_gain,
                            self.sensor_time,
                            self.sensor_ofst_lim_trans,
                            self.sensor_ofst_lim_rot,
                        ) = struct.unpack_from("<6i", buf, CMD_HEADER_SIZE)
                        buf = buf[needed:]
                        log.info("[RobotServer] CONFIGURATION updated.")

                    elif cmd in ("T", "D"):  # TERMINATE / DISCONNECT
                        buf = buf[CMD_HEADER_SIZE:]
                        self.robot_status_char = b"T"
                        log.info("[RobotServer] TERMINATE/DISCONNECT received.")

                    else:
                        log.warning("[RobotServer] Unknown command '%s' (0x%02X)", cmd, ord(cmd[0]))
                        buf = buf[CMD_HEADER_SIZE:]
        except Exception as e:
            log.warning("[RobotServer] Client error: %s", e)
        finally:
            writer.close()
            log.info("[RobotServer:1105] Client disconnected: %s", peer)

    async def _run_robot_server(self, host: str, port: int) -> None:
        """Start TCP server on port 1105 for robot commands."""
        server = await asyncio.start_server(self._handle_robot_client, host=host, port=port)
        log.info("Comau ROS 2 Robot Server listening on %s:%d", host, port)
        try:
            await server.serve_forever()
        finally:
            server.close()
            await server.wait_closed()

    # -----------------------------------------------------------------------
    # Motion Server (Port 1101)
    # -----------------------------------------------------------------------

    async def _handle_motion_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        peer = writer.get_extra_info("peername")
        log.info("[MotionServer:1101] Client connected: %s", peer)

        buf = b""
        try:
            while True:
                chunk = await reader.read(4096)
                if not chunk:
                    break
                buf += chunk

                while len(buf) >= CMD_HEADER_SIZE:
                    msg_id, motion_char = struct.unpack_from(CMD_HEADER_FMT, buf, 0)
                    mtype = motion_char.decode("ascii", errors="replace")

                    if mtype == "J":  # JOINT_POSITION
                        needed = CMD_HEADER_SIZE + 24  # 6 floats
                        if len(buf) < needed:
                            break
                        jnt_targets = list(struct.unpack_from("<6f", buf, CMD_HEADER_SIZE))
                        buf = buf[needed:]
                        log.info("[MotionServer] JOINT_POSITION target: %s", [f"{j:.2f}°" for j in jnt_targets])
                        await self._execute_joint_target(jnt_targets)

                    elif mtype == "F":  # JOINT_MOVEFLY
                        needed = CMD_HEADER_SIZE + 4 + 4 + 24  # lin_vel(4), fly_dist(4), 6f(24)
                        if len(buf) < needed:
                            break
                        lin_vel, fly_dist = struct.unpack_from("<2f", buf, CMD_HEADER_SIZE)
                        jnt_targets = list(struct.unpack_from("<6f", buf, CMD_HEADER_SIZE + 8))
                        buf = buf[needed:]
                        log.info("[MotionServer] MOVEFLY: vel=%.2f fly=%.2f target=%s",
                                 lin_vel, fly_dist, [f"{j:.2f}°" for j in jnt_targets])
                        await self._execute_joint_target(jnt_targets)

                    elif mtype == "T":  # JOINT_TRAJECTORY
                        if len(buf) < CMD_HEADER_SIZE + 4:
                            break
                        traj_size = struct.unpack_from("<I", buf, CMD_HEADER_SIZE)[0]
                        point_size = 40 + 4 + 4  # 10f(40) + seg_ovr(4) + move_type(4) = 48 bytes
                        needed = CMD_HEADER_SIZE + 4 + traj_size * point_size
                        if len(buf) < needed:
                            break
                        off = CMD_HEADER_SIZE + 4
                        points = []
                        for _ in range(traj_size):
                            raw_pts = struct.unpack_from("<10f", buf, off)
                            seg_ovr, move_type = struct.unpack_from("<Ii", buf, off + 40)
                            points.append(list(raw_pts[:6]))
                            off += point_size
                        buf = buf[needed:]
                        log.info("[MotionServer] JOINT_TRAJECTORY with %d points", traj_size)
                        if points:
                            await self._execute_trajectory(points)

                    elif mtype == "C":  # CARTESIAN_TRAJECTORY
                        if len(buf) < CMD_HEADER_SIZE + 4:
                            break
                        traj_size = struct.unpack_from("<I", buf, CMD_HEADER_SIZE)[0]
                        point_size = 24 + 4 + 4 + 4  # 6f(24) + lin_vel(4) + seg_ovr(4) + move_type(4) = 36 bytes
                        needed = CMD_HEADER_SIZE + 4 + traj_size * point_size
                        if len(buf) < needed:
                            break
                        off = CMD_HEADER_SIZE + 4
                        cart_points = []
                        for _ in range(traj_size):
                            pose = struct.unpack_from("<6f", buf, off)
                            lin_vel, seg_ovr, move_type = struct.unpack_from("<fIi", buf, off + 24)
                            cart_points.append(list(pose))
                            off += point_size
                        buf = buf[needed:]
                        log.info("[MotionServer] CARTESIAN_TRAJECTORY with %d points", traj_size)
                        # Target last waypoint via IK
                        if cart_points:
                            last_pose = cart_points[-1]
                            from .kinematics import inverse_kinematics
                            ik_sol = inverse_kinematics(last_pose, list(self._state.joint_pos))
                            if ik_sol:
                                await self._execute_joint_target(ik_sol)
                            else:
                                log.error("[MotionServer] IK failed for Cartesian trajectory endpoint")

                    elif mtype == "S":  # SENSOR_TRACKING
                        needed = CMD_HEADER_SIZE + 24
                        if len(buf) < needed:
                            break
                        corrections = struct.unpack_from("<6f", buf, CMD_HEADER_SIZE)
                        buf = buf[needed:]
                        log.debug("[MotionServer] SENSOR_TRACKING: %s", corrections)

                    else:
                        log.warning("[MotionServer] Unknown motion type '%s'", mtype)
                        buf = buf[CMD_HEADER_SIZE:]
        except Exception as e:
            log.warning("[MotionServer] Client error: %s", e)
        finally:
            writer.close()
            log.info("[MotionServer:1101] Client disconnected: %s", peer)

    async def _execute_joint_target(self, target: List[float]) -> None:
        """Trigger motion controller to interpolate to target."""
        self.robot_status_char = b"M"
        async with self._state.lock:
            self._state.motion_target_joints = list(target)
            self._state.motion_type = "JNT"
            self._state.motion_event.set()

    async def _execute_trajectory(self, waypoints: List[List[float]]) -> None:
        """Execute a multi-point trajectory."""
        self.robot_status_char = b"M"
        # For simplicity, move to endpoint
        if waypoints:
            await self._execute_joint_target(waypoints[-1])

    async def _run_motion_server(self, host: str, port: int) -> None:
        """Start TCP server on port 1101 for trajectory execution."""
        server = await asyncio.start_server(self._handle_motion_client, host=host, port=port)
        log.info("Comau ROS 2 Motion Server listening on %s:%d", host, port)
        try:
            await server.serve_forever()
        finally:
            server.close()
            await server.wait_closed()

    # -----------------------------------------------------------------------
    # Service Lifecycle
    # -----------------------------------------------------------------------

    async def start(self, host: str = "0.0.0.0") -> None:
        """Launch all 3 ROS 2 PDL servers as asyncio background tasks."""
        self._tasks.append(asyncio.create_task(
            self._run_state_server(host, config.ROS2_STATE_PORT), name="pdl_state_server"
        ))
        self._tasks.append(asyncio.create_task(
            self._run_robot_server(host, config.ROS2_ROBOT_PORT), name="pdl_robot_server"
        ))
        self._tasks.append(asyncio.create_task(
            self._run_motion_server(host, config.ROS2_MOTION_PORT), name="pdl_motion_server"
        ))
        log.info("All 3 Comau ROS 2 PDL servers started successfully.")

    async def stop(self) -> None:
        """Stop all 3 servers."""
        for t in self._tasks:
            t.cancel()
        for w in self._state_writers:
            try:
                w.close()
            except Exception:
                pass
        self._state_writers.clear()
        self._tasks.clear()
