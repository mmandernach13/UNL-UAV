#!/bin/bash

# Get the repo root (parent of scripts directory where this script lives)
REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

# Define paths relative to repo root
PX4_DIR="$REPO_ROOT/../PX4-Autopilot"
MICRO_DDS_DIR="$REPO_ROOT/../Micro-XRCE-DDS-Agent"
PARAMS_FILE="$REPO_ROOT/px4_configs/sim_no_gcs.txt"

echo "Starting simulator setup with tmux..."
echo "Repo root: $REPO_ROOT"

# Check if PX4 directory exists
if [ ! -d "$PX4_DIR" ]; then
    echo "ERROR: PX4-Autopilot directory not found at $PX4_DIR"
    exit 1
fi

# Check if Micro-DDS directory exists
if [ ! -d "$MICRO_DDS_DIR" ]; then
    echo "ERROR: Micro-XRCE-DDS-Agent directory not found at $MICRO_DDS_DIR"
    exit 1
fi

# Check if params file exists
if [ ! -f "$PARAMS_FILE" ]; then
    echo "WARNING: Params file not found at $PARAMS_FILE"
    echo "Continuing without setting custom params..."
    LOAD_PARAMS=false
else
    echo "Params file found: $PARAMS_FILE"
    LOAD_PARAMS=true
fi

if command -v tmux >/dev/null 2>&1; then
        echo "tmux is already installed."
else
    echo "tmux not found. Installing..."

    if [ -x "$(command -v apt)" ]; then
        sudo apt update && sudo apt install -y tmux
    elif [ -x "$(command -v dnf)" ]; then
        sudo dnf install -y tmux
    elif [ -x "$(command -v yum)" ]; then
        sudo yum install -y tmux
    elif [ -x "$(command -v pacman)" ]; then
        sudo pacman -Sy --noconfirm tmux
    elif [ -x "$(command -v brew)" ]; then
        brew install tmux
    else
        echo "No supported package manager found. Please install tmux manually."
        return 1
    fi

    echo "tmux installed successfully."
fi 

# Create a new tmux session named "uav_sim"
SESSION_NAME="uav_sim"

# Kill existing session if it exists
tmux has-session -t $SESSION_NAME 2>/dev/null
if [ $? -eq 0 ]; then
    echo "Killing existing session..."
    tmux kill-session -t $SESSION_NAME
fi

# Create new session with PX4 in first window
tmux new-session -d -s $SESSION_NAME -n "PX4" -c "$PX4_DIR"
tmux send-keys -t $SESSION_NAME:0 "make px4_sitl gz_x500" C-m

# Wait for PX4 to start
echo "Waiting for PX4 to start..."
sleep 8

# Load and set parameters from file
if [ "$LOAD_PARAMS" = true ]; then
    echo "Setting parameters from $PARAMS_FILE..."
    
    # Read file line by line
    while IFS= read -r line || [ -n "$line" ]; do
        # Skip empty lines and comments (lines starting with #)
        if [[ -z "$line" ]] || [[ "$line" =~ ^[[:space:]]*# ]]; then
            continue
        fi
        
        # Parse parameter name and value (format: "PARAM_NAME value")
        param_name=$(echo "$line" | awk '{print $1}')
        param_value=$(echo "$line" | awk '{print $2}')
        
        if [[ -n "$param_name" ]] && [[ -n "$param_value" ]]; then
            echo "  Setting $param_name = $param_value"
            tmux send-keys -t $SESSION_NAME:0 "param set $param_name $param_value" C-m
            sleep 0.3  # Small delay between commands
        fi
    done < "$PARAMS_FILE"
    
    # Save all parameters
    echo "Saving parameters..."
    sleep 0.5
    tmux send-keys -t $SESSION_NAME:0 "param save" C-m
    echo "Parameters set and saved!"
fi

# Create second window for Micro-DDS
tmux new-window -t $SESSION_NAME:1 -n "MicroDDS" -c "$MICRO_DDS_DIR/build"
tmux send-keys -t $SESSION_NAME:1 "./MicroXRCEAgent udp4 -p 8888" C-m

# Optional: Create third window back at repo root
tmux new-window -t $SESSION_NAME:2 -n "Workspace" -c "$REPO_ROOT/ros2_ws"
tmux send-keys -t $SESSION_NAME:2 "source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:2 "clear" C-m

# Attach to the session
echo "Attaching to tmux session '$SESSION_NAME'..."
echo "Use 'Ctrl+b' then number keys (0,1,2) to switch windows"
echo "Use 'Ctrl+b' then 'd' to detach (keeps running in background)"
tmux attach-session -t $SESSION_NAME