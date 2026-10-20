"""
robot_state.py - Thread-safe shared robot state for Comau Racer 5-0.80 SE simulator.

All access to mutable state goes through RobotState, which uses asyncio.Lock
to prevent race conditions between the TCP server and the motion controller.
"""

import asyncio
import enum
import time
from dataclasses import dataclass, field
from typing import List, Optional

from . import config


class ControllerMode(enum.Enum):
    """Operating mode of the Comau R1C-6 controller."""
    MANUAL = "MAN"
    AUTOMATIC = "AUTO"
    PROGRAM = "PROG"


class RobotStatus(enum.Enum):
    """High-level robot operational status."""
    IDLE = "IDLE"
    MOVING = "MOVING"
    HOLD = "HOLD"
    ERROR = "ERROR"
    ESTOP = "ESTOP"


@dataclass
class CartesianPose:
    """Cartesian TCP pose in world frame [mm, degrees]."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rx: float = 0.0   # rotation around X (roll)  [deg]
    ry: float = 0.0   # rotation around Y (pitch) [deg]
    rz: float = 0.0   # rotation around Z (yaw)   [deg]

    def to_list(self) -> List[float]:
        return [self.x, self.y, self.z, self.rx, self.ry, self.rz]


@dataclass
class ActiveAlarm:
    """A single active alarm entry."""
    code: int
    level: int
    description: str
    timestamp: float = field(default_factory=time.time)


class RobotState:
    """
    Central shared state of the simulated Comau controller.

    All fields are protected by a single asyncio.Lock. Callers must use:
        async with state.lock:
            state.field = ...
    """

    def __init__(self) -> None:
        self.lock = asyncio.Lock()

        # --- Joint positions / velocities [degrees, deg/s] ---
        self.joint_pos: List[float] = list(config.JOINT_HOME_DEG)
        self.joint_vel: List[float] = [0.0] * 6
        self.num_joints: int = 6

        # --- Cartesian TCP pose ---
        self.cartesian: CartesianPose = CartesianPose()

        # --- Digital I/O ---
        self.digital_out: List[bool] = [False] * config.NUM_DIGITAL_OUT
        self.digital_in:  List[bool] = [False] * config.NUM_DIGITAL_IN

        # --- Analog I/O [engineering units, 0-10V mapped 0.0-10.0] ---
        self.analog_out: List[float] = [0.0] * config.NUM_ANALOG_OUT
        self.analog_in:  List[float] = [0.0] * config.NUM_ANALOG_IN

        # --- Controller state ---
        self.mode: ControllerMode = ControllerMode.AUTOMATIC
        self.status: RobotStatus  = RobotStatus.IDLE
        self.drives_on: bool      = True
        self.estop_active: bool   = False
        self.speed_override: int  = 100   # percentage 0-100

        # --- Alarms ---
        self.alarms: List[ActiveAlarm] = []

        # --- Motion target (set by TCP server, consumed by motion controller) ---
        self.motion_target_joints: Optional[List[float]] = None
        self.motion_type: Optional[str] = None   # "JNT" or "LIN"
        self.motion_requested: bool = False
        self.motion_event: asyncio.Event = asyncio.Event()

        # --- Statistics ---
        self.rx_count: int = 0
        self.tx_count: int = 0
        self.connected_client: Optional[str] = None

        # --- Last heartbeat from client ---
        self.last_heartbeat: float = time.time()

    # ------------------------------------------------------------------
    # Convenience helpers (must be called with lock held by caller)
    # ------------------------------------------------------------------

    def raise_alarm(self, code: int, level: int, description: str) -> None:
        """Add alarm if not already present."""
        if not any(a.code == code for a in self.alarms):
            self.alarms.append(ActiveAlarm(code, level, description))
            if level >= 12:
                self.drives_on = False
                self.status = RobotStatus.ERROR
            elif level >= 8:
                self.status = RobotStatus.HOLD
            elif level >= 10:
                self.drives_on = False

    def clear_alarms(self) -> None:
        """Reset all alarms. Drives must be re-enabled manually."""
        self.alarms.clear()

    def has_blocking_alarm(self) -> bool:
        """Return True if any active alarm prevents motion."""
        return any(a.level >= 8 for a in self.alarms) or self.estop_active
