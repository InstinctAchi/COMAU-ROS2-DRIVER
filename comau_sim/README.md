# Comau Racer 5-0.80 SE / R1C-6 REL.2 – TCP/IP Robot Simulator

Realistic simulation of the **Comau Racer 5-0.80 SE** industrial robot with **R1C-6 REL.2** controller and **Camozzi CSPT** pneumatic gripper, exposed via TCP/IP binary telegram protocol (emulating the PDL2 Read/Write on TCP/IP driver).

## Features

- **Full kinematic simulation** – real DH parameters, joint limits (J1±170°…J6±2700°), trapezoidal velocity profile interpolation at 100 Hz
- **Binary TCP/IP protocol** – fixed-frame telegrams with STX header, sequence number, and XOR checksum (little-endian, C-packed), compatible with standard PLC/PC integrations
- **Complete I/O model** – 32 digital outputs, 32 digital inputs, 4 analog outputs, 8 analog inputs
- **Camozzi CSPT gripper** – mapped to DO[1]/DO[2] with DI[1]/DI[2] feedback, 300 ms pneumatic response delay
- **Comau C5G alarm system** – real error codes (20001, 20012, 10005…) with TP-style formatting (`XXXXX-LL`)
- **Forward and inverse kinematics** – FK (DH → cartesian), IK (damped least-squares Jacobian)
- **Live curses TUI dashboard** – real-time joint positions, cartesian pose, I/O, gripper state, alarms, client info

## Requirements

```
numpy>=1.24
```
No other dependencies. Python 3.10+.

## Install

```bash
cd comau_sim
pip install numpy
```

## Run

```bash
# With TUI dashboard (default)
python -m comau_sim.simulator

# Headless (logging only)
python -m comau_sim.simulator --no-tui

# Custom address/port
python -m comau_sim.simulator --host 0.0.0.0 --port 10000

# Debug logging
python -m comau_sim.simulator --no-tui --log-level DEBUG
```

Press **q** in the TUI to quit.

## Testing & Tools

### Interactive REPL

```bash
# Default (127.0.0.1:10000)
python tools/repl.py

# Custom host and port
python tools/repl.py --host 192.168.1.50 --port 10005
python tools/repl.py -H 127.0.0.1 -p 10005
python tools/repl.py 127.0.0.1 10005
```

### Real Robot TCP/IP Monitor (Terminal TUI)

Interactive curses terminal dashboard to monitor all TCP/IP ports and live 50 Hz telemetry of the real Comau C5G controller (or simulator):

```bash
# Monitor real robot at default IP (192.168.1.10)
python3 tools/monitor_robot.py

# Custom IP
python3 tools/monitor_robot.py 192.168.1.10
python3 tools/monitor_robot.py --host 192.168.1.10

# Single text snapshot (non-interactive / scriptable)
python3 tools/monitor_robot.py --once
```

**Keybindings in TUI:**
- `Q`: Quit
- `C`: Send Alarm Reset (`'R'`) to port 1105
- `S`: Send Start command (`'S'`) to port 1105
- `Space`: Pause / Resume live telemetry display


### Automated Integration Tests

```bash
# Terminal 1: start simulator
python -m comau_sim.simulator --no-tui

# Terminal 2: run test client (default 127.0.0.1:10000)
python tests/test_client.py

# With custom host and port
python tests/test_client.py --host 192.168.1.50 --port 10005
python tests/test_client.py -H 127.0.0.1 -p 10005
python tests/test_client.py 127.0.0.1 10005

# Test official Comau ROS 2 PDL server ports (1101, 1104, 1105)
python tests/test_ros2_pdl.py
```

## Comau ROS 2 Driver Emulation (COMAU-ROS2-DRIVER)

`comau_sim` natively emulates the official Comau PDL2 server programs (`comau_ros2_server`):
- **Port 1101**: `motion_server` (trajectories, Cartesian and joint moves)
- **Port 1104**: `state_server` (50 Hz binary telemetry stream, 173-byte packets)
- **Port 1105**: `robot_server` (lifecycle commands, pin IO)

Both the single-socket protocol (port 10000) and the ROS 2 multi-socket PDL servers (ports 1101, 1104, 1105) run concurrently.

To disable ROS 2 PDL servers:
```bash
python -m comau_sim.simulator --no-ros2
```

To connect the official ROS 2 driver to the local simulator:
```bash
# In robots/models/Comau/COMAU-ROS2-DRIVER:
./setup_ros2_workspace.sh --robot-ip 127.0.0.1
```

## Protocol Reference

### Telegram Frame

```
┌────────┬─────────┬──────────┬────────────┬───────────────┬──────────┐
│ STX    │ VERSION │ MSG_TYPE │ SEQ_NUM    │ PAYLOAD_LEN   │ PAYLOAD  │  CHECKSUM │
│ 1 byte │ 1 byte  │ 1 byte   │ 2 byte LE  │ 2 byte LE     │ N bytes  │  1 byte   │
└────────┴─────────┴──────────┴────────────┴───────────────┴──────────┘
STX      = 0x02
VERSION  = 0x01
CHECKSUM = XOR of bytes [VERSION..end of PAYLOAD]
All multi-byte integers: little-endian
Floats: IEEE 754 single precision (4 bytes)
```

### Client → Simulator Commands

| MSG_TYPE | Name | Payload |
|----------|------|---------|
| 0x01 | GET_STATE | empty |
| 0x02 | MOVE_JNT | 6×float32 joint angles [degrees] |
| 0x03 | MOVE_LIN | 6×float32 [X,Y,Z mm, Rx,Ry,Rz degrees] |
| 0x04 | SET_SPEED | 1×uint8 speed override 0-100% |
| 0x05 | SET_IO | uint8 channel, uint8 type (0=DO,1=AO), float32 value |
| 0x06 | GET_IO | empty |
| 0x07 | RESET_ALARM | empty |
| 0x08 | SET_MODE | 1×uint8 (0=MAN, 1=AUTO, 2=PROG) |
| 0x09 | ESTOP | 1×uint8 (1=engage, 0=release) |
| 0x0A | HEARTBEAT | empty |
| 0x0B | GRIPPER_CMD | 1×uint8 (0=open, 1=close) |
| 0x0C | GET_ALARMS | empty |
| 0x0D | HOME | empty |

### Simulator → Client Responses

| MSG_TYPE | Name | Payload |
|----------|------|---------|
| 0x80 | STATE_RESP | See below |
| 0x81 | ACK | empty |
| 0x82 | NACK | uint32 code, uint16 level, ASCII message |
| 0x83 | IO_RESP | bitmask DO(4B) + DI(4B) + AO(4×f32) + AI(8×f32) |
| 0x84 | ALARM_RESP | uint8 count + N×(uint32 code, uint16 level, uint16 rsvd, 32B desc) |
| 0x85 | HEARTBEAT_ACK | empty |
| 0x86 | MOTION_DONE | 6×float32 final joint positions [degrees] |

#### STATE_RESP Payload Layout

```
Offset  Size  Description
0       24    6×float32  joint positions [degrees]
24      24    6×float32  joint velocities [deg/s]
48      24    6×float32  cartesian [X mm, Y mm, Z mm, Rx°, Ry°, Rz°]
72       4    uint32     DO bitmask (DO[1]=bit0 … DO[32]=bit31)
76       4    uint32     DI bitmask
80      16    4×float32  analog outputs [0.0-10.0 V]
96      32    8×float32  analog inputs
128      1    uint8      status (0=IDLE,1=MOVING,2=HOLD,3=ERROR,4=ESTOP)
129      1    uint8      mode (0=MAN,1=AUTO,2=PROG)
130      1    uint8      drives_on (1/0)
131      1    uint8      estop (1/0)
132      1    uint8      speed_override (0-100)
133      1    uint8      alarm_count
134     16    ASCII      controller model (null-padded)
150     16    ASCII      robot model (null-padded)
```

### NACK Error Codes (Comau C5G style)

| Code  | Level | Description |
|-------|-------|-------------|
| 20001 | 12 | Emergency stop active |
| 20003 | 10 | Drive fault |
| 20012 | 10 | Joint position limit exceeded |
| 10005 |  8 | Heartbeat timeout |
| 30002 |  4 | IK failed: target unreachable |
| 60001 |  4 | Unknown MSG_TYPE |
| 60002 |  4 | Invalid payload |

## Joint Limits

| Axis | Min | Max | Max Speed |
|------|-----|-----|-----------|
| J1 | -170° | +170° | 360°/s |
| J2 | -64° | +114° | 300°/s |
| J3 | -150° | +60° | 330°/s |
| J4 | -200° | +200° | 500°/s |
| J5 | -105° | +100° | 500°/s |
| J6 | -2700° | +2700° | 800°/s |

## Gripper I/O Mapping (Camozzi CSPT)

| Signal | Index | Description |
|--------|-------|-------------|
| DO[1] | 0 | Open command |
| DO[2] | 1 | Close command |
| DI[1] | 0 | Open feedback (active after ~300 ms) |
| DI[2] | 1 | Close feedback (active after ~300 ms) |
