#!/bin/bash
#
# Launch SAR Multi-Drone Simulation
# PX4 SITL + Gazebo Harmonic + ROS2
#
# Usage: ./launch_simulation.bash [num_drones]
#

set -e

# ============ CONFIGURATION ============
NUM_DRONES=${1:-3}
PX4_DIR="${HOME}/PX4-Autopilot"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_FILE="${SCRIPT_DIR}/../worlds/sar_environment.sdf"

# Drone spawn positions (charging stations)
declare -a SPAWN_X=(-15 -15 -15)
declare -a SPAWN_Y=(-8 0 8)
SPAWN_Z=0.5

# ============ COLORS ============
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ============ FUNCTIONS ============
print_header() {
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

print_status() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

cleanup() {
    echo ""
    print_warning "Shutting down simulation..."
    
    # Kill all background processes
    pkill -f "px4" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "MicroXRCEAgent" 2>/dev/null || true
    pkill -f "ruby" 2>/dev/null || true  # Gazebo uses ruby
    
    print_status "Cleanup complete"
    exit 0
}

trap cleanup SIGINT SIGTERM

# ============ MAIN ============
print_header "SAR Multi-Drone Simulation"
echo "  Drones: ${NUM_DRONES}"
echo "  PX4 Dir: ${PX4_DIR}"
echo "  World: ${WORLD_FILE}"

# Check PX4
if [ ! -d "$PX4_DIR" ]; then
    print_error "PX4-Autopilot not found at $PX4_DIR"
    echo "  Install: git clone https://github.com/PX4/PX4-Autopilot.git --recursive"
    exit 1
fi

# Check Gazebo
if ! command -v gz &> /dev/null; then
    print_error "Gazebo (gz) not found"
    echo "  Install: sudo apt install gz-harmonic"
    exit 1
fi

# Check world file
if [ ! -f "$WORLD_FILE" ]; then
    print_warning "World file not found: $WORLD_FILE"
    print_warning "Using default empty world"
    WORLD_FILE=""
fi

# ============ STEP 1: Build PX4 if needed ============
print_header "Step 1: Checking PX4 Build"
cd "$PX4_DIR"

if [ ! -f "build/px4_sitl_default/bin/px4" ]; then
    print_warning "PX4 not built, building now (this takes a while)..."
    make px4_sitl_default
fi
print_status "PX4 build OK"

# ============ STEP 2: Start Gazebo ============
print_header "Step 2: Starting Gazebo"

# Set resource paths
export GZ_SIM_RESOURCE_PATH="${PX4_DIR}/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_RESOURCE_PATH="${SCRIPT_DIR}/../worlds:${GZ_SIM_RESOURCE_PATH}"

# Copy world to PX4 worlds if it exists
if [ -f "$WORLD_FILE" ]; then
    mkdir -p "${PX4_DIR}/Tools/simulation/gz/worlds"
    cp "$WORLD_FILE" "${PX4_DIR}/Tools/simulation/gz/worlds/sar_world.sdf"
    print_status "World file copied"
fi

# Start Gazebo
echo "Starting Gazebo Harmonic..."
if [ -f "$WORLD_FILE" ]; then
    gz sim -r "$WORLD_FILE" &
else
    gz sim -r empty.sdf &
fi
GZ_PID=$!
print_status "Gazebo started (PID: $GZ_PID)"

# Wait for Gazebo to initialize
echo "Waiting for Gazebo to initialize..."
sleep 8

# ============ STEP 3: Start Micro XRCE-DDS Agent ============
print_header "Step 3: Starting DDS Agent"

if command -v MicroXRCEAgent &> /dev/null; then
    MicroXRCEAgent udp4 -p 8888 &
    AGENT_PID=$!
    print_status "MicroXRCEAgent started (PID: $AGENT_PID)"
    sleep 2
else
    print_warning "MicroXRCEAgent not found - ROS2 communication disabled"
    print_warning "Install: See PX4 docs for Micro XRCE-DDS setup"
fi

# ============ STEP 4: Launch PX4 Instances ============
print_header "Step 4: Launching PX4 Drones"

declare -a PX4_PIDS

for ((i=0; i<NUM_DRONES; i++)); do
    echo ""
    echo -e "${BLUE}--- Drone $i ---${NC}"
    echo "  Position: (${SPAWN_X[$i]}, ${SPAWN_Y[$i]}, ${SPAWN_Z})"
    
    # Export environment for this instance
    export PX4_SYS_AUTOSTART=4001
    export PX4_SIM_MODEL=gz_x500
    export PX4_GZ_MODEL_POSE="${SPAWN_X[$i]},${SPAWN_Y[$i]},${SPAWN_Z},0,0,0"
    export PX4_INSTANCE=$i
    
    # Unique ports
    export MAVLINK_UDP_REMOTE_PORT=$((14550 + i))
    
    # Launch PX4
    cd "$PX4_DIR"
    ./build/px4_sitl_default/bin/px4 -i $i -d "build/px4_sitl_default/etc" > /dev/null 2>&1 &
    PX4_PIDS[$i]=$!
    
    print_status "Drone $i launched (PID: ${PX4_PIDS[$i]})"
    
    # Wait between spawns
    sleep 4
done

# ============ STEP 5: Summary ============
print_header "Simulation Ready!"

echo ""
echo "Drones are spawned and ready for control."
echo ""
echo "  QGroundControl: Connect to UDP localhost:14550"
echo ""
echo "  ROS2 Topics (if DDS agent running):"
echo "    /px4_0/fmu/out/vehicle_local_position"
echo "    /px4_1/fmu/out/vehicle_local_position"
echo "    /px4_2/fmu/out/vehicle_local_position"
echo ""
echo "  To launch coverage controller:"
echo "    ros2 launch sar_coverage sar_system.launch.py"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop simulation${NC}"
echo ""

# Wait for processes
wait
