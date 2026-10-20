"""
tcp_server.py - asyncio TCP server for the Comau R1C-6 PDL2 TCP/IP driver simulation.

Listens on TCP_PORT (default 10000). Each connected client gets its own
reader/writer task. Commands are dispatched, validated, and replied to
with the binary telegram format defined in protocol.py.

Heartbeat watchdog: if no MSG_HEARTBEAT is received within HEARTBEAT_TIMEOUT_S,
alarm 10005 is raised and the client connection is closed.
"""

import asyncio
import logging
import time
from typing import Callable, Coroutine, List, Optional

from . import config
from .alarm_manager import AlarmManager
from .gripper import GripperController
from .kinematics import check_joint_limits, inverse_kinematics
from . import protocol as P
from .robot_state import ControllerMode, RobotState, RobotStatus

log = logging.getLogger("comau.tcp")


class ClientSession:
    """Handles a single TCP client connection."""

    def __init__(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
        state: RobotState,
        alarm_mgr: AlarmManager,
        gripper: GripperController,
    ) -> None:
        self._reader  = reader
        self._writer  = writer
        self._state   = state
        self._alarm   = alarm_mgr
        self._gripper = gripper
        self._buf: bytes = b""
        self._seq: int = 0
        self._peer = writer.get_extra_info("peername", ("?", 0))
        self._motion_done_seq: int = 0

    # ------------------------------------------------------------------
    # Send helpers
    # ------------------------------------------------------------------

    async def _send(self, data: bytes) -> None:
        self._writer.write(data)
        await self._writer.drain()
        async with self._state.lock:
            self._state.tx_count += 1

    async def _ack(self, seq: int) -> None:
        await self._send(P.encode_ack(seq))

    async def _nack(self, seq: int, err_code: int, err_level: int, msg: str = "") -> None:
        await self._send(P.encode_nack(seq, err_code, err_level, msg))

    # ------------------------------------------------------------------
    # MOTION_DONE async callback (called by motion controller on completion)
    # ------------------------------------------------------------------

    async def notify_motion_done(self, final_joints: List[float]) -> None:
        try:
            await self._send(P.encode_motion_done(self._motion_done_seq, final_joints))
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Main receive/dispatch loop
    # ------------------------------------------------------------------

    async def run(self) -> None:
        peer_str = f"{self._peer[0]}:{self._peer[1]}"
        log.info("Client connected: %s", peer_str)
        async with self._state.lock:
            self._state.connected_client = peer_str
            self._state.last_heartbeat = time.time()

        try:
            while True:
                chunk = await asyncio.wait_for(
                    self._reader.read(4096),
                    timeout=config.HEARTBEAT_TIMEOUT_S + 1.0,
                )
                if not chunk:
                    break  # connection closed

                self._buf += chunk

                async with self._state.lock:
                    self._state.rx_count += 1
                    self._state.last_heartbeat = time.time()

                # Process all complete frames in buffer
                while True:
                    result = P.decode_frame(self._buf)
                    if result is None:
                        break
                    msg_type, seq_num, payload, self._buf = result
                    await self._dispatch(msg_type, seq_num, payload)

        except asyncio.TimeoutError:
            # Heartbeat timeout
            self._alarm.raise_alarm(10005)
            async with self._state.lock:
                self._state.status = RobotStatus.HOLD
            log.warning("Client %s heartbeat timeout", peer_str)
        except (ConnectionResetError, asyncio.IncompleteReadError):
            pass
        finally:
            try:
                self._writer.close()
                await self._writer.wait_closed()
            except Exception:
                pass
            async with self._state.lock:
                self._state.connected_client = None
            log.info("Client disconnected: %s", peer_str)

    # ------------------------------------------------------------------
    # Command dispatcher
    # ------------------------------------------------------------------

    async def _dispatch(self, msg_type: int, seq: int, payload: bytes) -> None:
        s = self._state
        alarm = self._alarm

        if msg_type == P.MSG_HEARTBEAT:
            await self._send(P.encode_heartbeat_ack(seq))
            return

        if msg_type == P.MSG_GET_STATE:
            await self._send_state(seq)
            return

        if msg_type == P.MSG_GET_IO:
            async with s.lock:
                do = list(s.digital_out)
                di = list(s.digital_in)
                ao = list(s.analog_out)
                ai = list(s.analog_in)
            await self._send(P.encode_io_resp(seq, do, di, ao, ai))
            return

        if msg_type == P.MSG_GET_ALARMS:
            alarms_list = alarm.active_list()
            from .robot_state import ActiveAlarm
            alarm_objs = [ActiveAlarm(c, l, d) for c, l, d in alarms_list]
            await self._send(P.encode_alarm_resp(seq, alarm_objs))
            return

        if msg_type == P.MSG_RESET_ALARM:
            alarm.clear_all()
            async with s.lock:
                s.alarms.clear()
                if not s.estop_active:
                    s.drives_on = True
                    if s.status in (RobotStatus.HOLD, RobotStatus.ERROR):
                        s.status = RobotStatus.IDLE
            await self._ack(seq)
            return

        if msg_type == P.MSG_ESTOP:
            engage = P.decode_estop(payload)
            if engage is None:
                await self._nack(seq, *P.ERR_INVALID_PAYLOAD, "ESTOP payload malformed")
                return
            async with s.lock:
                s.estop_active = engage
                if engage:
                    s.drives_on = False
                    s.status = RobotStatus.ESTOP
                    alarm.raise_alarm(20001)
                else:
                    alarm.clear_alarm(20001)
                    if not alarm.blocks_motion():
                        s.drives_on = True
                        s.status = RobotStatus.IDLE
            await self._ack(seq)
            return

        if msg_type == P.MSG_SET_MODE:
            mode_byte = P.decode_set_mode(payload)
            if mode_byte is None:
                await self._nack(seq, *P.ERR_INVALID_PAYLOAD)
                return
            modes = [ControllerMode.MANUAL, ControllerMode.AUTOMATIC, ControllerMode.PROGRAM]
            async with s.lock:
                s.mode = modes[mode_byte % 3]
            await self._ack(seq)
            return

        if msg_type == P.MSG_SET_SPEED:
            ovr = P.decode_set_speed(payload)
            if ovr is None or ovr > 100:
                await self._nack(seq, *P.ERR_SPEED_RANGE, "Speed 0-100 %")
                alarm.raise_alarm(40002, f"received={ovr}")
                return
            async with s.lock:
                s.speed_override = ovr
            await self._ack(seq)
            return

        if msg_type == P.MSG_SET_IO:
            result = P.decode_set_io(payload)
            if result is None:
                await self._nack(seq, *P.ERR_INVALID_PAYLOAD)
                return
            channel, io_type, value = result
            io_err = False
            async with s.lock:
                if io_type == 0 and 0 <= channel < config.NUM_DIGITAL_OUT:
                    s.digital_out[channel] = bool(value)
                elif io_type == 1 and 0 <= channel < config.NUM_ANALOG_OUT:
                    s.analog_out[channel] = float(value)
                else:
                    io_err = True
            if io_err:
                await self._nack(seq, *P.ERR_INVALID_PAYLOAD, f"channel={channel} type={io_type}")
            else:
                await self._ack(seq)
            return

        if msg_type == P.MSG_GRIPPER_CMD:
            cmd = P.decode_gripper_cmd(payload)
            if cmd is None:
                await self._nack(seq, *P.ERR_INVALID_PAYLOAD)
                return
            await self._ack(seq)  # ack immediately; gripper is async
            if cmd == 0:
                asyncio.ensure_future(self._gripper.open())
            else:
                asyncio.ensure_future(self._gripper.close())
            return

        if msg_type == P.MSG_HOME:
            await self._start_move_jnt(seq, config.JOINT_HOME_DEG)
            return

        if msg_type == P.MSG_MOVE_JNT:
            target = P.decode_move_jnt(payload)
            if target is None:
                await self._nack(seq, *P.ERR_INVALID_PAYLOAD, "MOVE_JNT needs 6 floats")
                return
            await self._start_move_jnt(seq, target)
            return

        if msg_type == P.MSG_MOVE_LIN:
            target_pose = P.decode_move_lin(payload)
            if target_pose is None:
                await self._nack(seq, *P.ERR_INVALID_PAYLOAD, "MOVE_LIN needs 6 floats")
                return
            await self._start_move_lin(seq, target_pose)
            return

        # Unknown message type
        alarm.raise_alarm(60001, f"msg_type=0x{msg_type:02X}")
        await self._nack(seq, *P.ERR_UNKNOWN_MSG, f"Unknown MSG_TYPE 0x{msg_type:02X}")

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------

    async def _start_move_jnt(self, seq: int, target: List[float]) -> None:
        s = self._state
        alarm = self._alarm

        err: Optional[tuple] = None
        async with s.lock:
            if alarm.blocks_motion() or s.estop_active:
                err = (P.ERR_BLOCKED[0], P.ERR_BLOCKED[1], "Drives off or alarm active")
            else:
                violated = check_joint_limits(target)
                if violated is not None:
                    alarm.raise_alarm(20012, f"J{violated + 1}={target[violated]:.2f}deg")
                    err = (P.ERR_JOINT_LIMIT[0], P.ERR_JOINT_LIMIT[1],
                           f"J{violated + 1} target {target[violated]:.2f} out of range")
                else:
                    s.motion_target_joints = list(target)
                    s.motion_type = "JNT"
                    s.motion_requested = True
                    self._motion_done_seq = seq
                    s.motion_event.set()

        if err:
            await self._nack(seq, err[0], err[1], err[2])
        else:
            await self._ack(seq)

    async def _start_move_lin(self, seq: int, target_pose: List[float]) -> None:
        s = self._state
        alarm = self._alarm

        err: Optional[tuple] = None
        current_joints: List[float] = []
        async with s.lock:
            if alarm.blocks_motion() or s.estop_active:
                err = (P.ERR_BLOCKED[0], P.ERR_BLOCKED[1], "Drives off or alarm active")
            else:
                current_joints = list(s.joint_pos)

        if err:
            await self._nack(seq, err[0], err[1], err[2])
            return

        # IK is CPU-bound, run outside lock
        ik_result = await asyncio.get_event_loop().run_in_executor(
            None, inverse_kinematics, target_pose, current_joints
        )

        if ik_result is None:
            alarm.raise_alarm(30002, f"target={target_pose}")
            await self._nack(seq, *P.ERR_PDL2_PROG, "IK failed: target unreachable")
            return

        err = None
        async with s.lock:
            violated = check_joint_limits(ik_result)
            if violated is not None:
                alarm.raise_alarm(20012, f"J{violated + 1} after IK")
                err = (P.ERR_JOINT_LIMIT[0], P.ERR_JOINT_LIMIT[1],
                       f"IK solution violates J{violated + 1} limit")
            else:
                s.motion_target_joints = ik_result
                s.motion_type = "LIN"
                self._motion_done_seq = seq
                s.motion_event.set()

        if err:
            await self._nack(seq, err[0], err[1], err[2])
        else:
            await self._ack(seq)

    # ------------------------------------------------------------------
    # State response helper
    # ------------------------------------------------------------------

    async def _send_state(self, seq: int) -> None:
        s = self._state
        alarm = self._alarm
        alarms_list = alarm.active_list()

        async with s.lock:
            status_map = {
                RobotStatus.IDLE:   0,
                RobotStatus.MOVING: 1,
                RobotStatus.HOLD:   2,
                RobotStatus.ERROR:  3,
                RobotStatus.ESTOP:  4,
            }
            mode_map = {
                ControllerMode.MANUAL:    0,
                ControllerMode.AUTOMATIC: 1,
                ControllerMode.PROGRAM:   2,
            }
            data = P.encode_state(
                seq_num         = seq,
                joint_pos       = list(s.joint_pos),
                joint_vel       = list(s.joint_vel),
                cartesian       = s.cartesian.to_list(),
                digital_out     = list(s.digital_out),
                digital_in      = list(s.digital_in),
                analog_out      = list(s.analog_out),
                analog_in       = list(s.analog_in),
                status          = status_map.get(s.status, 0),
                mode            = mode_map.get(s.mode, 1),
                drives_on       = s.drives_on,
                estop           = s.estop_active,
                speed_override  = s.speed_override,
                alarm_count     = len(alarms_list),
                controller_model= config.CONTROLLER_MODEL,
                robot_model     = config.ROBOT_MODEL,
            )
        await self._send(data)


# ---------------------------------------------------------------------------
# Server factory
# ---------------------------------------------------------------------------

async def run_server(
    state: RobotState,
    alarm_mgr: AlarmManager,
    gripper: GripperController,
    motion_done_registry: list,   # mutable list: server appends callback
) -> None:
    """Start the TCP server. Runs until cancelled."""

    async def client_connected(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        session = ClientSession(reader, writer, state, alarm_mgr, gripper)
        motion_done_registry.clear()
        motion_done_registry.append(session.notify_motion_done)
        await session.run()
        if motion_done_registry and motion_done_registry[0] is session.notify_motion_done:
            motion_done_registry.clear()

    server = await asyncio.start_server(
        client_connected,
        host=config.TCP_HOST,
        port=config.TCP_PORT,
        limit=65536,
    )
    addr = server.sockets[0].getsockname()
    log.info("Comau R1C-6 TCP server listening on %s:%s", addr[0], addr[1])

    async with server:
        await server.serve_forever()
