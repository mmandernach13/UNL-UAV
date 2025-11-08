#!/bin/bash

# Get the repo root (parent of scripts directory where this script lives)
REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

# Define paths relative to repo root
PX4_DIR="$REPO_ROOT/../PX4-Autopilot"
MICRO_DDS_DIR="$REPO_ROOT/../Micro-XRCE-DDS-Agent"

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

# Create second window for Micro-DDS
tmux new-window -t $SESSION_NAME:1 -n "MicroDDS" -c "$MICRO_DDS_DIR/build"
tmux send-keys -t $SESSION_NAME:1 "./MicroXRCEAgent udp4 -p 8888" C-m

# Optional: Create third window back at repo root
tmux new-window -t $SESSION_NAME:2 -n "Workspace" -c "$REPO_ROOT"

# Attach to the session
echo "Attaching to tmux session '$SESSION_NAME'..."
echo "Use 'Ctrl+b' then number keys (0,1,2) to switch windows"
echo "Use 'Ctrl+b' then 'd' to detach (keeps running in background)"
tmux attach-session -t $SESSION_NAME