#!/bin/bash

# Set plugin path untuk Betaflight plugin
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/nv/Documents/betaflight/betaflight_gazebo_plugin/build

echo "==================================================="
echo "Starting Gazebo with Betaflight Plugin"
echo "==================================================="
echo "Plugin path: $GAZEBO_PLUGIN_PATH"
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
