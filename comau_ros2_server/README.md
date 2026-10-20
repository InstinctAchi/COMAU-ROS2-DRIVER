# Comau ROS 2 Server PDL2 Files

These PDL2 programs run on the physical Comau robot controller (C5G, C5GPlus, R1C) or in the Comau RoboShop virtual controller simulation.

## Overview

| Program | Type | Description |
|---|---|---|
| `pdl_tcp_functions.pdl` | NOHOLD | Master program: initializes shared variables and loads all other programs. |
| `state_server.pdl` | NOHOLD | TCP server on **port 1104**: transmits 50 Hz telemetry stream to ROS 2. |
| `robot_server.pdl` | NOHOLD | TCP server on **port 1105**: processes control commands (`INITIALIZE`, `START`, `RESET`, `IO`, etc.). |
| `motion_server.pdl` | NOHOLD | TCP server on **port 1101**: receives trajectory waypoints and feeds the motion queue. |
| `arm1_handler.pdl` | HOLD | Executes physical arm motion commands (`MOVE`) on ARM[1]. |

## Deployment on Physical Robot

For the complete step-by-step physical robot guide, compilation instructions, FTP transfer, and Teach Pendant procedures, see:

👉 [Physical Robot Deployment Guide (README_PHYSICAL_ROBOT.md)](../comau_ros2_client/comau_driver/README_PHYSICAL_ROBOT.md)

## Automated Deployment Script

To automatically transfer all server files to your physical controller:
```bash
cd ..
./deploy_to_robot.sh --robot-ip 192.168.1.50
```
