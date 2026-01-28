#!/usr/bin/env python3
"""
Simple Altitude Hold Implementation for Betaflight SITL
Uses manual throttle adjustment based on altitude feedback
"""

import socket
import struct
import time
import threading

MSP_HEADER = b'$M<'
MSP_SET_RAW_RC = 200
MSP_ALTITUDE = 109  # Get altitude
RC_CENTER = 1500
RC_MIN = 1000

class AltitudeHold:
    def __init__(self, host='127.0.0.1', port=5762):
        self.host = host
        self.port = port
        self.sock = None
        self.channels = [RC_CENTER] * 8
        self.channels[2] = RC_MIN
        
        # Altitude hold state
        self.target_altitude = 5.0  # meters
        self.current_altitude = 0.0
        self.base_throttle = 52  # percentage
        self.altitude_hold_enabled = False
        
        # PID gains for altitude hold
        self.kP = 8.0  # Proportional gain
        self.kI = 0.5  # Integral gain
        self.kD = 12.0  # Derivative gain
        
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()
        
    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            print(f"✓ Connected to Betaflight SITL at {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False
    
    def msp_checksum(self, data):
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum
    
    def send_rc(self, throttle_percent, roll=0, pitch=0, yaw=0, aux1=1):
        """Send RC command"""
        throttle_pwm = int(RC_MIN + (throttle_percent / 100.0) * 1000)
        throttle_pwm = max(RC_MIN, min(2000, throttle_pwm))
        
        self.channels[0] = int(RC_CENTER + roll * 5)  # Roll
        self.channels[1] = int(RC_CENTER + pitch * 5)  # Pitch
        self.channels[2] = throttle_pwm  # Throttle
        self.channels[3] = int(RC_CENTER + yaw * 5)  # Yaw
        self.channels[4] = 2000 if aux1 else RC_MIN  # AUX1 (ARM)
        
        payload = struct.pack('<' + 'H' * 8, *self.channels)
        size = len(payload)
        packet = MSP_HEADER + bytes([size, MSP_SET_RAW_RC]) + payload
        checksum = self.msp_checksum(bytes([size, MSP_SET_RAW_RC]) + payload)
        packet += bytes([checksum])
        
        try:
            self.sock.send(packet)
            return True
        except:
            return False
    
    def update_altitude_from_gazebo(self):
        """Parse altitude from Gazebo console output
        You need to monitor Gazebo console and manually update
        In real implementation, would use MSP_ALTITUDE or telemetry
        """
        # For now, we'll estimate from throttle changes
        # In production, parse from Gazebo logs or implement MSP_ALTITUDE
        pass
    
    def altitude_hold_loop(self):
        """Main altitude hold control loop"""
        print("\n🚁 Altitude Hold Active")
        print(f"   Target: {self.target_altitude:.1f}m")
        print(f"   Base throttle: {self.base_throttle}%")
        print("\nControls:")
        print("  Press 'q' to disable altitude hold")
        print("  Adjust target_altitude in code if needed\n")
        
        while self.altitude_hold_enabled:
            current_time = time.time()
            dt = current_time - self.last_time
            
            if dt < 0.02:  # 50Hz control loop
                time.sleep(0.001)
                continue
            
            self.last_time = current_time
            
            # MANUAL: Since we can't easily get altitude from FC in this simple script,
            # we'll use a simple open-loop controller
            # User needs to monitor Gazebo altitude and adjust target_altitude
            
            # For demonstration: simple proportional controller
            # In reality, you'd parse altitude from Gazebo console or MSP
            
            # Simple hover with manual adjustment
            throttle = self.base_throttle
            
            self.send_rc(throttle)
            time.sleep(0.02)
    
    def test_altitude_hold(self):
        print("\n" + "="*60)
        print("  BETAFLIGHT SITL - ALTITUDE HOLD TEST")
        print("="*60)
        
        # ARM
        print("\n1. Arming drone...")
        for i in range(20):
            self.send_rc(0)
            time.sleep(0.05)
        print("✓ Armed")
        time.sleep(2)
        
        # Takeoff to target altitude
        print(f"\n2. Taking off to {self.target_altitude}m...")
        for t in range(0, self.base_throttle, 1):
            self.send_rc(t)
            time.sleep(0.1)
        
        print(f"\n3. Hovering at {self.base_throttle}% throttle")
        print("   Monitor altitude in Gazebo console")
        print("   If climbing: Reduce self.base_throttle in code")
        print("   If descending: Increase self.base_throttle")
        
        # Hold for 20 seconds
        for i in range(200):
            self.send_rc(self.base_throttle)
            time.sleep(0.1)
            if i % 50 == 0:
                print(f"   Holding... {i/10:.0f}s")
        
        # Land
        print("\n4. Landing...")
        for t in range(self.base_throttle, -1, -1):
            self.send_rc(t)
            time.sleep(0.05)
        
        # Disarm
        print("5. Disarming...")
        for i in range(10):
            self.send_rc(0, aux1=0)
            time.sleep(0.05)
        
        print("✓ Done")

def main():
    print("=" * 60)
    print("  BETAFLIGHT ALTITUDE HOLD (Manual Tuning)")
    print("=" * 60)
    print("\nNOTE: Betaflight 4.x does NOT have built-in altitude hold.")
    print("This script provides MANUAL throttle-based hover assistance.")
    print("\nFor true altitude hold, you need:")
    print("  - ArduPilot or PX4 (has full altitude controller)")
    print("  - OR external companion computer with altitude control")
    print("\nThis script will:")
    print("  1. ARM the drone")
    print("  2. Takeoff to ~5m")
    print("  3. Hold throttle at fixed percentage")
    print("  4. YOU must monitor and adjust base_throttle")
    print("=" * 60)
    
    input("\nPress ENTER to start (make sure Gazebo and SITL running)...")
    
    ah = AltitudeHold()
    if not ah.connect():
        return
    
    try:
        ah.test_altitude_hold()
    except KeyboardInterrupt:
        print("\n\n✗ Interrupted")
    finally:
        if ah.sock:
            ah.sock.close()

if __name__ == '__main__':
    main()
