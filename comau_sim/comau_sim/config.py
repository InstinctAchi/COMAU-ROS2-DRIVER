"""
config.py - Comau Racer 5-0.80 SE / R1C-6 REL.2 simulator configuration.

All physical parameters are from the official Comau Racer 5-0.80 datasheet.
Network defaults match the PDL2 TCP/IP driver typical installation (port 10000).
"""

import math

# ---------------------------------------------------------------------------
# Network
# ---------------------------------------------------------------------------
TCP_HOST = "0.0.0.0"
TCP_PORT = 10000
MAX_CLIENTS = 4
HEARTBEAT_TIMEOUT_S = 5.0   # seconds before raising alarm 10005

# ---------------------------------------------------------------------------
# Official Comau ROS 2 Driver (COMAU-ROS2-DRIVER) PDL2 Server emulation ports
# Matches comau_ros2_server: motion_server=1101, state_server=1104, robot_server=1105
# ---------------------------------------------------------------------------
ROS2_ENABLE_PDL_SERVERS = True
ROS2_MOTION_PORT = 1101     # TCP port for Arm1 motion server (motion_server.pdl)
ROS2_STATE_PORT = 1104      # TCP port for state server (state_server.pdl)
ROS2_ROBOT_PORT = 1105      # TCP port for robot/executor server (robot_server.pdl)
ROS2_STATE_PUBLISH_HZ = 50.0 # State publishing frequency [Hz]

# ---------------------------------------------------------------------------
# Protocol (Port 10000 single-socket custom protocol)
# ---------------------------------------------------------------------------
PROTOCOL_STX = 0x02
PROTOCOL_VERSION = 0x01
MOTION_DONE_NOTIFY = True   # send async MOTION_DONE telegram when move ends

# ---------------------------------------------------------------------------
# Controller identity strings (as exposed in GET_STATE response)
# ---------------------------------------------------------------------------
CONTROLLER_MODEL = "R1C-6"
CONTROLLER_RELEASE = "REL.2"
ROBOT_MODEL = "Racer5-0.80-SE"
ROBOT_SERIAL = "SIM-0001"

# ---------------------------------------------------------------------------
# Denavit-Hartenberg parameters for Comau Racer 5-0.80
#   Each row: (a_mm, d_mm, alpha_rad, theta_offset_rad)
#   Conventions: modified DH (Craig), theta_i = joint_i + offset_i
# ---------------------------------------------------------------------------
DH_PARAMS = [
    # a [mm]   d [mm]   alpha [rad]          theta_offset [rad]
    (150.0,    235.0,   -math.pi / 2,        0.0),           # J1
    (590.0,    0.0,      0.0,                -math.pi / 2),   # J2
    (130.0,    0.0,     -math.pi / 2,        math.pi / 2),   # J3
    (0.0,      647.0,    math.pi / 2,        0.0),           # J4
    (0.0,      0.0,     -math.pi / 2,        0.0),           # J5
    (0.0,      95.0,     0.0,                0.0),           # J6
]

# ---------------------------------------------------------------------------
# Joint limits [degrees]  - Racer 5-0.80 official datasheet
# ---------------------------------------------------------------------------
JOINT_LIMITS_DEG = [
    (-170.0,  170.0),    # J1
    (-64.0,   114.0),    # J2
    (-150.0,   60.0),    # J3
    (-200.0,  200.0),    # J4
    (-105.0,  100.0),    # J5
    (-2700.0, 2700.0),   # J6
]

# Maximum joint speeds [deg/s] - Racer 5-0.80 official datasheet
JOINT_MAX_SPEED_DEG_S = [360.0, 300.0, 330.0, 500.0, 500.0, 800.0]

# Typical acceleration [deg/s²] - estimated from Racer 5 dynamics
JOINT_ACCEL_DEG_S2 = [1000.0, 800.0, 900.0, 1200.0, 1200.0, 2000.0]

# Home position [degrees] - robot in vertical upright configuration
# All joints at 0° is within all joint limits
JOINT_HOME_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Motion controller update rate [Hz]
MOTION_LOOP_HZ = 100

# ---------------------------------------------------------------------------
# I/O configuration
# ---------------------------------------------------------------------------
NUM_DIGITAL_OUT = 32
NUM_DIGITAL_IN  = 32
NUM_ANALOG_OUT  = 4
NUM_ANALOG_IN   = 8

# Gripper I/O mapping (Camozzi CSPT via robot digital I/O)
GRIPPER_DO_OPEN  = 0   # index 0 → DO[1] in Comau notation (1-based on TP)
GRIPPER_DO_CLOSE = 1   # index 1 → DO[2]
GRIPPER_DI_OPEN  = 0   # index 0 → DI[1]
GRIPPER_DI_CLOSE = 1   # index 1 → DI[2]
GRIPPER_RESPONSE_DELAY_S = 0.30   # 300 ms pneumatic response
