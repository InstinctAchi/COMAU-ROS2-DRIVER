#!/usr/bin/env bash
# Starts a standalone MoveIt Servo node (test only), for planners launched with servo:=false.
#   run_servo.sh            -> DRY: the output goes to /servo_test/dry_joint_trajectory (the robot receives nothing)
#   run_servo.sh --real     -> the output goes to /joint_trajectory_controller/joint_trajectory (the robot moves)
# Run inside the robot-planner container.
set -e
PKG="${PKG:-rospackage_d5f6eada}"
PRJ="${PRJ:-PRJ043}"
CONFIG="/ws/projects/projects/${PRJ}/manipulator/${PKG}/config"
OUT_TOPIC="/servo_test/dry_joint_trajectory"
SCALE_LIN="${SCALE_LIN:-0.15}"
for arg in "$@"; do
  case "$arg" in
    --real) OUT_TOPIC="/joint_trajectory_controller/joint_trajectory" ;;
  esac
done
source /opt/ros/jazzy/setup.bash
source /ws/ros_overlay/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=/ws/cyclonedds.xml
# Since 2026-10-16 the planner launch starts its own servo_node (used by /robot/follow_tf):
# servo_follow_test.py works with it directly, a second node with the same name must not be started
if ros2 node list 2>/dev/null | grep -qx /servo_node; then
  echo "/servo_node already running (planner launch): use it, no standalone Servo needed" >&2
  exit 1
fi
PARAMS=/tmp/servo_test_params.yaml
python3 "$(dirname "$0")/gen_servo_params.py" --config-dir "$CONFIG" --out "$PARAMS" \
  --command-out-topic "$OUT_TOPIC" --scale-linear "$SCALE_LIN"
echo "Servo output topic: $OUT_TOPIC"
exec ros2 run moveit_servo servo_node --ros-args --params-file "$PARAMS" -r __node:=servo_node
