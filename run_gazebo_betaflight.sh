#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Set plugin path untuk Betaflight plugin dan Gazebo default plugins
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$SCRIPT_DIR/betaflight_gazebo_plugin/build:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins

# Set model path agar Gazebo menemukan model lokal (PENTING!)
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$SCRIPT_DIR/models

echo "==================================================="
echo "Starting Gazebo with Betaflight Plugin"
echo "==================================================="
echo "Plugin path: $GAZEBO_PLUGIN_PATH"
echo "Model path: $GAZEBO_MODEL_PATH"
echo ""
echo "Ports:"
echo "  - Port 9003: Sending FDM data to Betaflight SITL"
echo "  - Port 9002: Receiving PWM data from Betaflight SITL"
echo ""
echo "Make sure Betaflight SITL is running!"
echo "==================================================="
echo ""

# Run Gazebo with the Betaflight world file
gazebo --verbose iris_betaflight_v2.world
