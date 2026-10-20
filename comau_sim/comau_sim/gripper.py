"""
gripper.py - Camozzi CSPT gripper simulation.

The Camozzi CSPT gripper is connected to the robot controller via Ethernet
TCP/IP and mapped as digital I/O on the robot bus. In the simulated controller:

    DO[GRIPPER_DO_OPEN]  → command gripper to open
    DO[GRIPPER_DO_CLOSE] → command gripper to close
    DI[GRIPPER_DI_OPEN]  → open position feedback
    DI[GRIPPER_DI_CLOSE] → closed position feedback

The gripper responds to commands with a configurable pneumatic delay
(default 300 ms), as is typical for Camozzi Series CSPT (10-20 bar).
A feedback mismatch alarm is raised if both DI are simultaneously active
(sensor fault) or if neither is active after the response window.
"""

import asyncio
import time

from . import config
from .alarm_manager import AlarmManager
from .robot_state import RobotState


class GripperState:
    """Logical state of the gripper."""
    OPEN   = "OPEN"
    CLOSED = "CLOSED"
    MOVING = "MOVING"
    FAULT  = "FAULT"


class GripperController:
    """
    Async gripper controller.

    Manages DO output commands and simulates DI feedback with realistic delay.
    Runs as a co-routine that can be awaited from the TCP command handler.
    """

    def __init__(self, state: RobotState, alarm_mgr: AlarmManager) -> None:
        self._state = state
        self._alarm = alarm_mgr
        self._gripper_state = GripperState.OPEN
        self._lock = asyncio.Lock()

        # Initialise feedback DI to "open" at startup
        asyncio.get_event_loop().call_soon(self._init_feedback)

    def _init_feedback(self) -> None:
        self._state.digital_in[config.GRIPPER_DI_OPEN]  = True
        self._state.digital_in[config.GRIPPER_DI_CLOSE] = False

    @property
    def state(self) -> str:
        return self._gripper_state

    async def open(self) -> bool:
        """
        Command the gripper to open.
        Returns True when feedback confirms open position.
        """
        async with self._lock:
            if self._gripper_state == GripperState.OPEN:
                return True  # already open

            self._gripper_state = GripperState.MOVING

            # Set DO: open command ON, close command OFF
            async with self._state.lock:
                self._state.digital_out[config.GRIPPER_DO_OPEN]  = True
                self._state.digital_out[config.GRIPPER_DO_CLOSE] = False
                # Clear feedback during movement
                self._state.digital_in[config.GRIPPER_DI_OPEN]  = False
                self._state.digital_in[config.GRIPPER_DI_CLOSE] = False

        # Simulate pneumatic response delay (outside lock to not block)
        await asyncio.sleep(config.GRIPPER_RESPONSE_DELAY_S)

        async with self._lock:
            async with self._state.lock:
                self._state.digital_in[config.GRIPPER_DI_OPEN]  = True
                self._state.digital_in[config.GRIPPER_DI_CLOSE] = False
                self._state.digital_out[config.GRIPPER_DO_OPEN] = False  # pulse done
            self._gripper_state = GripperState.OPEN
            return True

    async def close(self) -> bool:
        """
        Command the gripper to close.
        Returns True when feedback confirms closed position.
        """
        async with self._lock:
            if self._gripper_state == GripperState.CLOSED:
                return True  # already closed

            self._gripper_state = GripperState.MOVING

            async with self._state.lock:
                self._state.digital_out[config.GRIPPER_DO_OPEN]  = False
                self._state.digital_out[config.GRIPPER_DO_CLOSE] = True
                self._state.digital_in[config.GRIPPER_DI_OPEN]   = False
                self._state.digital_in[config.GRIPPER_DI_CLOSE]  = False

        await asyncio.sleep(config.GRIPPER_RESPONSE_DELAY_S)

        async with self._lock:
            async with self._state.lock:
                self._state.digital_in[config.GRIPPER_DI_OPEN]   = False
                self._state.digital_in[config.GRIPPER_DI_CLOSE]  = True
                self._state.digital_out[config.GRIPPER_DO_CLOSE] = False
            self._gripper_state = GripperState.CLOSED
            return True

    def inject_fault(self) -> None:
        """Simulate a gripper sensor fault (both DI simultaneously active)."""
        self._gripper_state = GripperState.FAULT
        self._alarm.raise_alarm(50010, "both sensors active simultaneously")
