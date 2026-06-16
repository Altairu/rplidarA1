#!/usr/bin/env bash
set -euo pipefail

# One-command launcher for rplidarA1 localization stack.
# Usage examples:
#   ./run_slam.sh
#   ./run_slam.sh --serial /dev/ttyUSB0 --namespace rplidar_localization
#   ./run_slam.sh --enable-drive

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}/ros2_ws"

SERIAL_PORT="/dev/ttyUSB0"
CAN_CHANNEL="can0"
NAMESPACE="rplidar_localization"
ENABLE_DRIVE="false"
ENABLE_REALSENSE_IMU="false"
RS_SERIAL_NO=""

print_help() {
  cat <<'EOF'
run_slam.sh - Launch SLAM + localization monitor in one shell

Options:
  --serial <path>       LiDAR serial port (default: /dev/ttyUSB0)
  --can <ifname>        CAN interface (default: can0)
  --namespace <name>    ROS namespace (default: rplidar_localization)
  --enable-drive        Enable mdd_can_node (default: disabled)
  --enable-realsense-imu Enable D435i IMU integration (default: disabled)
  --no-realsense-imu    Disable D435i IMU integration
  --rs-serial <serial>  RealSense serial number (optional)
  -h, --help            Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --serial)
      SERIAL_PORT="${2:-}"
      shift 2
      ;;
    --can)
      CAN_CHANNEL="${2:-}"
      shift 2
      ;;
    --namespace)
      NAMESPACE="${2:-}"
      shift 2
      ;;
    --enable-drive)
      ENABLE_DRIVE="true"
      shift
      ;;
    --no-realsense-imu)
      ENABLE_REALSENSE_IMU="false"
      shift
      ;;
    --enable-realsense-imu)
      ENABLE_REALSENSE_IMU="true"
      shift
      ;;
    --rs-serial)
      RS_SERIAL_NO="${2:-}"
      shift 2
      ;;
    -h|--help)
      print_help
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      print_help
      exit 1
      ;;
  esac
done

if [[ -z "${SERIAL_PORT}" || -z "${CAN_CHANNEL}" || -z "${NAMESPACE}" ]]; then
  echo "Required option value is empty." >&2
  print_help
  exit 1
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble setup not found: /opt/ros/humble/setup.bash" >&2
  exit 1
fi

if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  echo "Workspace setup not found: ${WS_DIR}/install/setup.bash" >&2
  echo "Build first: cd ${WS_DIR} && colcon build --symlink-install" >&2
  exit 1
fi

# ROS setup scripts may reference variables that are intentionally unset.
# Keep strict mode overall, but relax nounset only while sourcing.
set +u
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u

# Workaround for some Intel/Mesa environments where RViz map shader fails.
export LIBGL_ALWAYS_SOFTWARE=1

echo "Stopping stale SLAM/visualization processes (if any)..."
pkill -f "ros2 launch altair_robot robot_drive_launch.py" 2>/dev/null || true
pkill -f "sllidar_ros2/sllidar_node" 2>/dev/null || true
pkill -f "cartographer_ros/cartographer_node" 2>/dev/null || true
pkill -f "cartographer_occupancy_grid_node" 2>/dev/null || true
pkill -f "altair_robot/websocket_bridge_node" 2>/dev/null || true
pkill -f "altair_robot/map_marker_node" 2>/dev/null || true
pkill -f "rviz2 -d .*robot_drive.rviz" 2>/dev/null || true
pkill -f "python3 -m http.server 8091" 2>/dev/null || true

# Free serial device if another process still holds it.
if command -v fuser >/dev/null 2>&1; then
  fuser -k "${SERIAL_PORT}" >/dev/null 2>&1 || true
fi

if [[ "${ENABLE_REALSENSE_IMU}" == "true" ]]; then
  if pgrep -f "realsense2_camera_node" >/dev/null; then
    echo "Stale realsense2_camera_node detected. Terminating old process..."
    pkill -f "realsense2_camera_node" || true
  fi
fi

# Kill any stale processes on web/websocket ports to avoid "Address already in use" errors
echo "Clearing port 8091 (HTTP) and 8876 (WebSocket) from stale processes..."
if command -v fuser >/dev/null 2>&1; then
  fuser -k 8091/tcp 8876/tcp >/dev/null 2>&1 || true
fi
sleep 1

echo "Starting SLAM stack with:"
echo "  serial_port=${SERIAL_PORT}"
echo "  can_channel=${CAN_CHANNEL}"
echo "  namespace=${NAMESPACE}"
echo "  enable_drive=${ENABLE_DRIVE}"
echo "  enable_realsense_imu=${ENABLE_REALSENSE_IMU}"
if [[ -n "${RS_SERIAL_NO}" ]]; then
  echo "  rs_serial_no=${RS_SERIAL_NO}"
fi
echo "  Web UI: http://<robot-ip>:8091"
echo "  WebSocket: ws://<robot-ip>:8876"

launch_args=(
  "serial_port:=${SERIAL_PORT}"
  "can_channel:=${CAN_CHANNEL}"
  "namespace:=${NAMESPACE}"
  "enable_drive:=${ENABLE_DRIVE}"
  "enable_realsense_imu:=${ENABLE_REALSENSE_IMU}"
)

if [[ -n "${RS_SERIAL_NO}" ]]; then
  launch_args+=("rs_serial_no:=${RS_SERIAL_NO}")
fi

exec ros2 launch altair_robot robot_drive_launch.py "${launch_args[@]}"
