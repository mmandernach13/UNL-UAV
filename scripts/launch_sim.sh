# Default values
WORLD="default"
MODEL="unl_x500"
PX4_AIRFRAME="gz_x500"
SPAWN_X=0
SPAWN_Y=0
SPAWN_Z=0.5

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -w|--world)
            WORLD="$2"
            shift 2
            ;;
        -m|--model)
            MODEL="$2"
            shift 2
            ;;
        -a|--airframe)
            PX4_AIRFRAME="$2"
            shift 2
            ;;
        -x)
            SPAWN_X="$2"
            shift 2
            ;;
        -y)
            SPAWN_Y="$2"
            shift 2
            ;;
        -z)
            SPAWN_Z="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: launch_sim.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -w, --world WORLD      World file name (default: default)"
            echo "  -m, --model MODEL      Model name (default: unl_x500)"
            echo "  -a, --airframe FRAME   PX4 airframe (default: gz_x500)"
            echo "  -x X                   Spawn X position (default: 0)"
            echo "  -y Y                   Spawn Y position (default: 0)"
            echo "  -z Z                   Spawn Z position (default: 0.5)"
            echo "  -h, --help             Show this help message"
            echo ""
            echo "Examples:"
            echo "  ./launch_sim.sh"
            echo "  ./launch_sim.sh -w delivery_world -m unl_x500"
            echo "  ./launch_sim.sh -m unl_x500_camera -x 5 -y 10"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

# Set up paths
REPO_ROOT="$HOME/uav/unl_uav_repo"
MODELS_PATH="$REPO_ROOT/ros2_ws/src/uav_models/models"
WORLDS_PATH="$REPO_ROOT/ros2_ws/src/uav_models/worlds"
PX4_PATH="$HOME/PX4-Autopilot"

# Export Gazebo resource paths
export GZ_SIM_RESOURCE_PATH="$MODELS_PATH:$WORLDS_PATH:$GZ_SIM_RESOURCE_PATH"

# Determine world file path
if [ "$WORLD" = "default" ]; then
    WORLD_FILE="$PX4_PATH/Tools/simulation/gz/worlds/default.sdf"
else
    WORLD_FILE="$WORLDS_PATH/${WORLD}.sdf"
fi

# Check if world file exists
if [ ! -f "$WORLD_FILE" ]; then
    echo "Error: World file not found: $WORLD_FILE"
    exit 1
fi

# Check if model exists
if [ ! -d "$MODELS_PATH/$MODEL" ]; then
    echo "Error: Model not found: $MODELS_PATH/$MODEL"
    exit 1
fi

echo "========================================="
echo "UNL UAV Simulation Launcher"
echo "========================================="
echo "World:     $WORLD"
echo "Model:     $MODEL"
echo "Airframe:  $PX4_AIRFRAME"
echo "Position:  X=$SPAWN_X Y=$SPAWN_Y Z=$SPAWN_Z"
echo "========================================="

# Kill any existing Gazebo/PX4 instances
echo "Cleaning up existing instances..."
pkill -9 gz 2>/dev/null
pkill -9 px4 2>/dev/null
sleep 2

# Launch Gazebo with the specified world
echo "Launching Gazebo with world: $WORLD"
gz sim -r "$WORLD_FILE" &
GZ_PID=$!

# Wait for Gazebo to initialize
echo "Waiting for Gazebo to start..."
sleep 5

# Spawn the model
echo "Spawning model: $MODEL at position ($SPAWN_X, $SPAWN_Y, $SPAWN_Z)"
gz model --spawn-file="$MODELS_PATH/$MODEL/model.sdf" \
    -x $SPAWN_X -y $SPAWN_Y -z $SPAWN_Z \
    -n "${MODEL}_0"

if [ $? -ne 0 ]; then
    echo "Error: Failed to spawn model"
    kill $GZ_PID
    exit 1
fi

# Start PX4 SITL
echo "Starting PX4 SITL with airframe: $PX4_AIRFRAME"
cd "$PX4_PATH"
PX4_SIM_MODEL=$PX4_AIRFRAME make px4_sitl none_iris

# Cleanup on exit
trap "pkill -9 gz; pkill -9 px4" EXIT