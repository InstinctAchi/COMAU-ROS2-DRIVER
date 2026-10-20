# Comau Physical Robot Deployment Guide: C5G / C5GPlus / R1C Controller

Step-by-step procedure to deploy, compile, and run the official `COMAU-ROS2-DRIVER` server PDL2 programs on a physical Comau robot controller.

---

## 1. Controller Requirements & Prerequisites

### Hardware & Firmware
- **Supported Controllers**: Comau C5G, C5GPlus, R1C / R1C-6 (Open C5G architecture).
- **Supported Robots**: Racer 5-0.80, Racer 7, NJ, NS, AURA, and other 6-axis Comau manipulators.
- **Software Options**: The controller MUST have the **TCP/IP** software option enabled in its system license.
  - To verify on Teach Pendant: `Menu` -> `Setup` / `System` -> `Installed Options` -> look for `TCP/IP`.

### Network Configuration
1. Connect an industrial Ethernet cable from your ROS 2 computer (or switch) to the controller's external network port (typically `ETH1` or `ETH2` on the controller rack).
2. Configure a static IP address on the controller:
   - Teach Pendant: `Menu` -> `Setup` -> `Network Settings`.
   - Set IP (e.g. `192.168.1.50`), Subnet Mask (`255.255.255.0`), Gateway (`192.168.1.1`).
3. Set your PC network interface to the same subnet (e.g. `192.168.1.100`, mask `255.255.255.0`).
4. Verify connection from your PC:
   ```bash
   ping -c 3 192.168.1.50
   ```

---

## 2. Server PDL2 Files Overview

The server code is located in:
`robots/models/Comau/COMAU-ROS2-DRIVER/comau_ros2_server/`

It consists of 5 files:
| File | Role | Port | Execution Mode |
|---|---|---|---|
| `pdl_tcp_functions.pdl` | Master loader, global shared variables, utility routines | - | `NOHOLD` |
| `state_server.pdl` | Streams joint positions, cartesian pose, I/O at 50 Hz | **1104** | `NOHOLD` |
| `robot_server.pdl` | Handles lifecycle commands (`I`, `S`, `R`, `X`, `L`, `U`, `O`, `T`) | **1105** | `NOHOLD` |
| `motion_server.pdl` | Receives trajectory points and motion commands (`J`, `T`, `F`, `C`) | **1101** | `NOHOLD` |
| `arm1_handler.pdl` | Executes physical arm motion commands on ARM[1] | - | `HOLD` |

---

## 3. Compilation (PDL -> COD)

Comau controllers execute compiled bytecode (`.cod` files).

### Method A: Using Comau RoboShop / WinC5G (Recommended)
1. Open your robot project in **Comau RoboShop** on a Windows workstation.
2. Import the 5 `.pdl` files into the project.
3. Right-click on each file in the project tree and select **Translate** (or **Build All**):
   - `pdl_tcp_functions.pdl` -> `pdl_tcp_functions.cod`
   - `state_server.pdl` -> `state_server.cod`
   - `robot_server.pdl` -> `robot_server.cod`
   - `motion_server.pdl` -> `motion_server.cod`
   - `arm1_handler.pdl` -> `arm1_handler.cod`
4. Ensure there are 0 translation errors.

### Method B: Directly on Controller via Teach Pendant
1. Transfer the `.pdl` text files to the controller folder (see Step 4).
2. On Teach Pendant: `Menu` -> `Program Management` -> `Translate`.
3. Select each file and press **Execute Translate**.

---

## 4. File Transfer via FTP to Controller

The master program `pdl_tcp_functions` expects the files to reside in:
`UD:\USR\comau_ros2_server\`

### Automated Transfer:
Run our provided automated script:
```bash
./deploy_to_robot.sh --robot-ip 192.168.1.50
```

### Manual Transfer (FileZilla or CLI `ftp`):
```bash
ftp 192.168.1.50
# User: comau (or controller username)
# Password: <press enter if no password, or controller password>
ftp> mkdir UD:\USR\comau_ros2_server
ftp> cd UD:\USR\comau_ros2_server
ftp> binary
ftp> mput *.pdl *.cod
ftp> bye
```

Verify that all `.pdl` and `.cod` files exist in `UD:\USR\comau_ros2_server\`.

---

## 5. Starting the Server on the Physical Robot

### Safety Checks BEFORE Starting
1. Confirm no personnel are inside the robot cell.
2. Ensure the robot cell perimeter safety fence and interlocks are closed.
3. Keep the Teach Pendant Emergency Stop button accessible.
4. Set speed override on Teach Pendant to **10%** for initial tests.

### Execution Procedure on Teach Pendant (TP):
1. **Clear Safety Interlocks**:
   - Release E-Stop buttons.
   - Clear any active alarms on TP (`Alarm` -> `Reset All`).
2. **Switch Controller Mode**:
   - Turn the key switch to **AUTO** (or **REMOTE** if cell PLC handles drive-on).
3. **Energize Servos (Drive-On)**:
   - Press the **DRIVES ON** button on the operator panel (servos energized, brakes release).
4. **Load and Execute Program**:
   - Open Program List on TP.
   - Select `pdl_tcp_functions.cod` (DO NOT launch the individual servers manually).
   - Press the green **START** button.
5. **Verify Initialization**:
   - Look at the Teach Pendant user CRT terminal screen. You should see:
     ```
     [STATE_SERVER ] Server Port : 1104
     [ROBOT_SERVER ] Server Port : 1105
     [MTION_SERVER ] Server Port : 1101
     Waiting for connections...
     ```
   - The robot is now listening on ports 1101, 1104, 1105.

---

## 6. Connecting from the ROS 2 PC

1. **Configure Target IP**:
   Edit `comau_bringup/config/general_config.yaml`:
   ```yaml
   execute_trajectory_handler_node:
     ros__parameters:
       robot_ip: "192.168.1.50"      # IP of your physical robot
       arm1_server_port: "1101"
       state_server_port: "1104"
       robot_server_port: "1105"
   ```

2. **Launch Client Node**:
   ```bash
   source install/setup.bash
   ros2 launch comau_bringup start_comau_client.launch.py robot_type:=racer5-0-80
   ```

3. **Open TCP Connection**:
   In another terminal:
   ```bash
   ros2 service call /tcpip_conn_manager comau_msgs/srv/OpenConnection "open_connection: true"
   ```

4. **Verify Telemetry**:
   ```bash
   ros2 topic echo /robot_status
   ros2 topic echo /joint_states
   ```

---

## 7. Troubleshooting & Error Codes

| Symptom / Error Code | Cause | Solution |
|---|---|---|
| `KI_ERR_STATE_ACCEPT (0x00001) / Error 15470: Address in use` | A previous socket was left open or program restarted abruptly. | Deactivate all programs (`DEACTIVATE ALL`), wait 10s for TCP `TIME_WAIT` to expire, re-run `pdl_tcp_functions`. |
| `Cannot connect to 192.168.1.50:1104` | Robot program not started or firewall blocking ports. | Verify `pdl_tcp_functions` is running on TP, check `ping`, ensure ports 1101, 1104, 1105 are open. |
| `KI_ERR_STATE_SAFETY_GATE (0x08000)` | Safety gate / E-Stop opened while server active. | Close safety gate, reset alarms, re-enable Drives On. |
| `Robot does not move when trajectory sent` | Robot not in START/DRIVE-ON state. | Call `ros2 service call /set_arm_state comau_msgs/srv/SetArmState "arm_state: 2"` (Reset), then ensure green START is engaged. |
