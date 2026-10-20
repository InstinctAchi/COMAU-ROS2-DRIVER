#!/usr/bin/env bash
# ==============================================================================
# deploy_to_robot.sh
# Automated FTP Deployment Script for Comau C5G / C5GPlus / R1C Controllers
#
# Uploads comau_ros2_server PDL2 and COD files to UD:\USR\comau_ros2_server\
#
# Usage:
#   ./deploy_to_robot.sh --robot-ip 192.168.1.50 [--user comau] [--pass secret]
# ==============================================================================

set -euo pipefail

ROBOT_IP=""
FTP_USER="${COMAU_FTP_USER:-comau}"
FTP_PASS="${COMAU_FTP_PASS:-}"   # prefer the env var so the password stays out of the shell history
FTP_PORT=21
DRY_RUN=false
TARGET_DIR="UD:\\USR\\comau_ros2_server"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVER_DIR="${SCRIPT_DIR}/comau_ros2_server"

print_help() {
    cat << EOF
Usage: ./deploy_to_robot.sh [options]

Required:
  -i, --robot-ip <IP>      IP address of the physical Comau robot controller

Options:
  -u, --user <USER>        FTP username (default: comau)
  -p, --pass <PASS>        FTP password (default: empty)
  --port <PORT>            FTP port (default: 21)
  -t, --target-dir <DIR>   Target folder on the controller (default: UD:\USR\comau_ros2_server)
  --dry-run                Simulate transfer without uploading
  -h, --help               Show this help message

Example:
  ./deploy_to_robot.sh --robot-ip 192.168.1.50
EOF
}

# Parse CLI arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -i|--robot-ip)
            ROBOT_IP="$2"
            shift 2
            ;;
        -u|--user)
            FTP_USER="$2"
            shift 2
            ;;
        -p|--pass)
            FTP_PASS="$2"
            shift 2
            ;;
        --port)
            FTP_PORT="$2"
            shift 2
            ;;
        -t|--target-dir)
            TARGET_DIR="$2"
            shift 2
            ;;
        --dry-run)
            DRY_RUN=true
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

if [[ -z "${ROBOT_IP}" ]]; then
    echo "ERROR: --robot-ip is required." >&2
    print_help
    exit 1
fi

if [[ ! -d "${SERVER_DIR}" ]]; then
    echo "ERROR: Directory not found: ${SERVER_DIR}" >&2
    exit 1
fi

echo "=================================================================="
echo "  Deploying COMAU-ROS2-DRIVER to Robot Controller: ${ROBOT_IP}"
echo "=================================================================="

# 1. Connectivity test
echo -n "[1/4] Checking network reachability to ${ROBOT_IP}... "
if ping -c 1 -W 2 "${ROBOT_IP}" > /dev/null 2>&1; then
    echo "OK (Host reachable)"
else
    echo "WARNING: ping to ${ROBOT_IP} timed out. Proceeding anyway..."
fi

# 2. Check files to deploy
echo "[2/4] Locating server files in ${SERVER_DIR}..."
FILES_TO_UPLOAD=()
for f in "${SERVER_DIR}"/*.pdl "${SERVER_DIR}"/*.cod; do
    if [[ -f "$f" ]]; then
        FILES_TO_UPLOAD+=("$(basename "$f")")
        echo "  - $(basename "$f")"
    fi
done

if [[ ${#FILES_TO_UPLOAD[@]} -eq 0 ]]; then
    echo "ERROR: No .pdl or .cod files found in ${SERVER_DIR}" >&2
    exit 1
fi

# 3. Transfer via FTP
echo "[3/4] Uploading files to controller directory: ${TARGET_DIR}..."
if [[ "${DRY_RUN}" == "true" ]]; then
    echo "  [DRY RUN] Skipping actual FTP transfer."
else
    # Create temporary FTP batch script
    TMP_FTP="$(mktemp)"
    cat << EOF > "${TMP_FTP}"
open ${ROBOT_IP} ${FTP_PORT}
user ${FTP_USER} ${FTP_PASS}
binary
mkdir ${TARGET_DIR}
cd ${TARGET_DIR}
EOF
    for file in "${FILES_TO_UPLOAD[@]}"; do
        echo "put ${SERVER_DIR}/${file} ${file}" >> "${TMP_FTP}"
    done
    echo "quit" >> "${TMP_FTP}"

    # Try transfer using ftp or curl or python ftplib fallback
    if command -v ftp > /dev/null 2>&1; then
        ftp -n -v < "${TMP_FTP}" || true
    else
        # Python ftplib fallback (always available)
        python3 - << PYEOF
import os, sys
from ftplib import FTP

ip = "${ROBOT_IP}"
user = "${FTP_USER}"
passwd = "${FTP_PASS}"
server_dir = "${SERVER_DIR}"
target_dir = "${TARGET_DIR}"

print(f"Connecting to FTP on {ip}...")
try:
    ftp = FTP()
    ftp.connect(ip, ${FTP_PORT}, timeout=10)
    ftp.login(user, passwd)
    try:
        ftp.mkd(target_dir)
    except Exception:
        pass
    try:
        ftp.cwd(target_dir)
    except Exception as e:
        print(f"Warning: could not cwd to {target_dir}: {e}")

    for fname in os.listdir(server_dir):
        if fname.endswith(".pdl") or fname.endswith(".cod"):
            fpath = os.path.join(server_dir, fname)
            with open(fpath, "rb") as fp:
                ftp.storbinary(f"STOR {fname}", fp)
                print(f"  Uploaded: {fname}")
    ftp.quit()
    print("FTP transfer completed successfully.")
except Exception as e:
    print(f"FTP error: {e}", file=sys.stderr)
    sys.exit(1)
PYEOF
    fi
    rm -f "${TMP_FTP}"
fi

# 4. Final instructions
echo ""
echo "=================================================================="
echo "  DEPLOYMENT COMPLETE"
echo "=================================================================="
echo "Next steps on physical Teach Pendant (TP):"
echo "  1. If .cod files are not compiled yet, compile all .pdl on TP"
echo "  2. Energize motors: turn switch to AUTO and press DRIVES ON"
echo "  3. Select program: pdl_tcp_functions.cod"
echo "  4. Press green START button on Teach Pendant"
echo "  5. Verify listening ports on controller: 1101, 1104, 1105"
echo "=================================================================="
