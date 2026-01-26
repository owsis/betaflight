#!/bin/bash
# Script untuk menjalankan Betaflight SITL

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo "=================================================="
echo "  Betaflight SITL + Gazebo 11"
echo "=================================================="
echo ""
echo "Port yang akan digunakan:"
echo "  TCP 5760-5769: UART1-10 (MSP/CLI/Telemetry)"
echo "  UDP 9002: SITL -> Gazebo (Motor PWM commands)"
echo "  UDP 9003: Gazebo -> SITL (IMU/Sensor data)"
echo "  UDP 9004: RC input (optional)"
echo ""
echo "Petunjuk:"
echo "  1. Jalankan script ini di terminal 1"
echo "  2. Tunggu hingga SITL ready (muncul 'Armed' status)"
echo "  3. Jalankan ./run_gazebo.sh di terminal 2"
echo "  4. Connect dengan Betaflight Configurator jika diperlukan"
echo ""
echo "Memulai Betaflight SITL..."
echo ""

# Jalankan SITL
./obj/main/betaflight_SITL.elf
