#!/usr/bin/env bash
# ==============================================================================
# setup_ros2_workspace.sh
# Automated Build and Configuration Script for Comau ROS 2 Client
#
# Usage:
#   ./setup_ros2_workspace.sh [--robot-ip 127.0.0.1] [--robot-type racer5-0-80]
# ==============================================================================

set -euo pipefail

TARGET_IP="127.0.0.1"
ROBOT_TYPE="racer5-0-80"
BUILD_ONLY=false

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLIENT_DIR="${SCRIPT_DIR}/comau_ros2_client"
CONFIG_FILE="${CLIENT_DIR}/comau_bringup/config/general_config.yaml"

print_help() {
    cat << EOF
Usage: ./setup_ros2_workspace.sh [options]

Options:
  --robot-ip <IP>          Target robot IP (default: 127.0.0.1 for local comau_sim)
  --robot-type <TYPE>      Robot type (default: racer5-0-80)
  --build-only             Only run colcon build without launching
  -h, --help               Show this help message

Example:
  # Setup for local simulation:
  ./setup_ros2_workspace.sh --robot-ip 127.0.0.1

  # Setup for physical robot:
  ./setup_ros2_workspace.sh --robot-ip 192.168.1.50
EOF
}

# Parse CLI arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --robot-ip)
            TARGET_IP="$2"
            shift 2
            ;;
        --robot-type)
            ROBOT_TYPE="$2"
            shift 2
            ;;
        --build-only)
            BUILD_ONLY=true
            shift
            ;;
        -h|--help)
            print_help
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            print_help
            exit 1
            ;;
    esac
done

echo "=================================================================="
echo "  Setting up Comau ROS 2 Client Workspace"
echo "  Target IP:   ${TARGET_IP}"
echo "  Robot Type:  ${ROBOT_TYPE}"
echo "=================================================================="

# 1. Source ROS 2
echo "[1/4] Checking ROS 2 environment..."
if [[ -f "/opt/ros/humble/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    echo "  Sourced /opt/ros/humble/setup.bash"
elif [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    echo "  Sourced /opt/ros/jazzy/setup.bash"
elif compgen -G "/opt/ros/*/setup.bash" > /dev/null; then
    ROS_SETUP=(/opt/ros/*/setup.bash)
    # shellcheck disable=SC1090
    source "${ROS_SETUP[0]}"
    echo "  Sourced ${ROS_SETUP[0]}"
elif [[ -n "${ROS_DISTRO:-}" ]]; then
    echo "  ROS environment active (${ROS_DISTRO})"
else
    echo "WARNING: ROS 2 setup.bash not found. Ensure ROS 2 (Humble or Jazzy) is installed."
fi

# 2. Update general_config.yaml with target IP
echo "[2/4] Updating configuration: ${CONFIG_FILE}..."
if [[ -f "${CONFIG_FILE}" ]]; then
    sed -i -E "s/robot_ip: *\"[^\"]*\"/robot_ip: \"${TARGET_IP}\"/" "${CONFIG_FILE}"
    echo "  robot_ip set to \"${TARGET_IP}\""
else
    echo "WARNING: Config file not found at ${CONFIG_FILE}"
fi

# 3. Build workspace using colcon
echo "[3/4] Building ROS 2 packages in ${CLIENT_DIR}..."
cd "${CLIENT_DIR}"
if command -v colcon > /dev/null 2>&1; then
    colcon build --symlink-install
    echo "  Build completed successfully."
else
    echo "ERROR: 'colcon' command not found. Please install: sudo apt install python3-colcon-common-extensions" >&2
    exit 1
fi

# 4. Ready to run instructions
echo ""
echo "=================================================================="
echo "  SETUP COMPLETED SUCCESSFULLY"
echo "=================================================================="
echo "To run the driver, execute in your terminal:"
echo ""
echo "  cd ${CLIENT_DIR}"
echo "  source install/setup.bash"
echo "  ros2 launch comau_bringup start_comau_client.launch.py robot_type:=${ROBOT_TYPE}"
echo ""
echo "In another terminal, open the TCP connection:"
echo "  ros2 service call /tcpip_conn_manager comau_msgs/srv/OpenConnection \"open_connection: true\""
echo ""
echo "Monitor robot state:"
echo "  ros2 topic echo /robot_status"
echo "  ros2 topic echo /joint_states"
echo "=================================================================="
