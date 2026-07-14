#!/usr/bin/env bash
# ============================================================
# launch_vision.sh — Launch PX4 SITL + Gazebo (with rendering)
#                     for vision target detection
#
# Camera stream: UDP port 5600 (H264 via GStreamer)
# MAVLink:       udp://localhost:14540
#
# Usage:
#   bash launch_vision.sh
# ============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${HOME}/PX4-Autopilot"
BUILD_PATH="${PX4_DIR}/build/px4_sitl_default"

GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${CYAN}╔══════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║   🎯 Vision Drone — Target Follow Launcher  ║${NC}"
echo -e "${CYAN}╚══════════════════════════════════════════════╝${NC}"
echo ""

# ── Preflight checks ──
if [ ! -d "$PX4_DIR" ]; then
    echo -e "${RED}❌ PX4-Autopilot not found at ${PX4_DIR}${NC}"
    exit 1
fi
echo -e "${GREEN}✅ PX4-Autopilot found${NC}"

if ! command -v gazebo &> /dev/null; then
    echo -e "${RED}❌ Gazebo Classic not found${NC}"
    exit 1
fi
echo -e "${GREEN}✅ Gazebo Classic found${NC}"

# Build if needed
if [ ! -f "${BUILD_PATH}/bin/px4" ]; then
    echo -e "${YELLOW}Building PX4 SITL (first time only)...${NC}"
    cd "${PX4_DIR}"
    DONT_RUN=1 make px4_sitl gazebo-classic
fi
echo -e "${GREEN}✅ PX4 SITL built${NC}"

# Kill any old processes
pkill -x gazebo 2>/dev/null || true
pkill -x gzserver 2>/dev/null || true
pkill -x gzclient 2>/dev/null || true
pkill -x px4 2>/dev/null || true
sleep 2

# ── Source environments ──
if [ -f "/usr/share/gazebo-11/setup.bash" ]; then
    source /usr/share/gazebo-11/setup.bash
fi
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}✅ ROS 2 Humble sourced${NC}"
fi

# PX4 gazebo setup (adds plugin/model paths, LD_LIBRARY_PATH)
source "${PX4_DIR}/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
       "${PX4_DIR}" "${BUILD_PATH}" 2>/dev/null || true

# ── Add our custom models ──
export GAZEBO_MODEL_PATH="${SCRIPT_DIR}/gazebo/models:${GAZEBO_MODEL_PATH}"

# Force X11 backend for Qt (Wayland compat)
export QT_QPA_PLATFORM=xcb

# PX4 env vars
export PX4_SIM_MODEL=gazebo-classic_iris

WORLD_FILE="${SCRIPT_DIR}/gazebo/worlds/vision_targets.world"
MODEL_PATH=""

# Find the iris model
IFS_bak=$IFS
IFS=":"
for p in ${GAZEBO_MODEL_PATH}; do
    p=$(echo "$p" | tr -d '\r')
    if [ -f "${p}/iris/iris.sdf" ]; then
        MODEL_PATH="${p}/iris/iris.sdf"
        break
    fi
done
IFS=$IFS_bak

if [ -z "$MODEL_PATH" ]; then
    echo -e "${RED}❌ Iris model not found in GAZEBO_MODEL_PATH${NC}"
    exit 1
fi
echo -e "${GREEN}✅ Iris model: ${MODEL_PATH}${NC}"

echo ""
echo -e "${CYAN}Camera stream: UDP port 5600${NC}"
echo -e "${CYAN}MAVLink:       udp://localhost:14540${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop${NC}"
echo ""

# ── Step 1: Launch Gazebo WITH rendering (camera needs it) ──
echo -e "${GREEN}[1/3] Starting Gazebo with vision targets world...${NC}"
gazebo "${WORLD_FILE}" --verbose &
GAZEBO_PID=$!
echo "     Gazebo PID: ${GAZEBO_PID}"

# Wait for gazebo to be ready
echo -e "${YELLOW}     Waiting for Gazebo to initialize...${NC}"
sleep 8

# ── Step 2: Spawn iris_fpv_cam model ──
echo -e "${GREEN}[2/3] Spawning Iris with FPV camera...${NC}"

# Find the custom iris_vision_cam model first
FPV_MODEL=""
IFS_bak=$IFS
IFS=":"
for p in ${GAZEBO_MODEL_PATH}; do
    p=$(echo "$p" | tr -d '\r')
    if [ -f "${p}/iris_vision_cam/model.sdf" ]; then
        FPV_MODEL="${p}/iris_vision_cam/model.sdf"
        break
    elif [ -f "${p}/iris_fpv_cam/iris_fpv_cam.sdf" ]; then
        FPV_MODEL="${p}/iris_fpv_cam/iris_fpv_cam.sdf"
    fi
done
IFS=$IFS_bak

if [ -z "$FPV_MODEL" ]; then
    echo -e "${YELLOW}     iris_vision_cam / iris_fpv_cam not found, using standard iris${NC}"
    FPV_MODEL="${MODEL_PATH}"
fi

echo "     Model: ${FPV_MODEL}"

while gz model --verbose --spawn-file="${FPV_MODEL}" --model-name=iris -x 1.01 -y 0.98 -z 0.83 2>&1 | grep -q "An instance of Gazebo is not running."; do
    echo "     gzserver not ready yet, retrying..."
    sleep 1
done
echo -e "${GREEN}     Drone spawned!${NC}"
sleep 2

# ── Step 3: Start PX4 SITL ──
echo -e "${GREEN}[3/3] Starting PX4 SITL...${NC}"
ROOTFS="${BUILD_PATH}/rootfs/vision_cam"
mkdir -p "${ROOTFS}"
cd "${ROOTFS}"
"${BUILD_PATH}/bin/px4" -d "${BUILD_PATH}/etc" &
PX4_PID=$!
echo "     PX4 PID: ${PX4_PID}"

echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║   ✅ All systems launched!                   ║${NC}"
echo -e "${GREEN}║   Camera → UDP 5600                         ║${NC}"
echo -e "${GREEN}║   MAVLink → udp://:14540                    ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════╝${NC}"
echo ""

# ── Cleanup ──
cleanup() {
    echo -e "\n${YELLOW}Shutting down...${NC}"
    kill $PX4_PID 2>/dev/null || true
    kill $GAZEBO_PID 2>/dev/null || true
    killall -9 gzserver gzclient px4 2>/dev/null || true
    echo -e "${GREEN}Done.${NC}"
}
trap cleanup SIGINT SIGTERM EXIT

wait $GAZEBO_PID
