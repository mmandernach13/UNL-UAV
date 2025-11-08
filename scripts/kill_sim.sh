#!/bin/bash

echo "Stopping all UAV simulator processes..."

# Kill tmux session if it exists
if tmux has-session -t uav_sim 2>/dev/null; then
    echo "Killing tmux session 'uav_sim'..."
    tmux kill-session -t uav_sim
fi

# Kill PX4 processes
echo "Killing PX4 processes..."
pkill -f px4

# Kill Ruby simulation processes (from PX4)
echo "Killing simulation processes..."
pkill -f ruby

# Kill Micro-XRCE-DDS Agent
echo "Killing Micro-XRCE-DDS Agent..."
pkill -f MicroXRCEAgent

# Kill Gazebo if running
echo "Killing Gazebo..."
pkill -f gazebo
pkill -f gzserver
pkill -f gzclient

# Give processes time to clean up
sleep 2

echo "All simulator processes stopped!"

# Check if anything is still running
echo ""
echo "Checking for remaining processes..."
ps aux | grep -E "px4|ruby|MicroXRCE|gazebo" | grep -v grep

if [ $? -eq 0 ]; then
    echo ""
    echo "Warning: Some processes may still be running above."
else
    echo "All clean!"
fi