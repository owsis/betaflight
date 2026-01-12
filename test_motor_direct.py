#!/usr/bin/env python3
"""
Test motor control langsung dengan MSP_SET_MOTOR
PENTING: Hanya bekerja saat drone DISARMED!
"""

import socket
import struct
import time
import sys

MSP_STATUS = 101
MSP_SET_MOTOR = 214
MSP_MOTOR = 104
MSP_DISARM = 151  # Force disarm

class MotorTester:
    def __init__(self, host='127.0.0.1', port=5762):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5)
            self.sock.connect((self.host, self.port))
            print(f"✓ Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False

    def disconnect(self):
        if self.sock:
            self.sock.close()

    def _checksum(self, cmd, payload):
        checksum = len(payload) ^ cmd
        for byte in payload:
            checksum ^= byte
        return checksum

    def send_command(self, cmd, payload=b''):
        if not self.sock:
            return None

        header = b'$M<'
        size = len(payload)
        checksum = self._checksum(cmd, payload)
        packet = header + struct.pack('BB', size, cmd) + payload + struct.pack('B', checksum)

        try:
            self.sock.send(packet)

            # Read response
            response = b''
            while len(response) < 3 or response[0:3] != b'$M>':
                byte = self.sock.recv(1)
                if not byte:
                    return None
                response += byte
                if len(response) > 1000:
                    return None

            size_byte = self.sock.recv(1)
            cmd_byte = self.sock.recv(1)

            if not size_byte or not cmd_byte:
                return None

            resp_size = struct.unpack('B', size_byte)[0]
            resp_payload = b''
            if resp_size > 0:
                resp_payload = self.sock.recv(resp_size)

            resp_checksum = self.sock.recv(1)
            return resp_payload

        except Exception as e:
            print(f"✗ Send error: {e}")
            return None

    def get_status(self):
        payload = self.send_command(MSP_STATUS)
        if not payload or len(payload) < 11:
            return None

        flight_mode_flags = struct.unpack('<I', payload[6:10])[0]
        armed = bool(flight_mode_flags & 0x1)
        return {'armed': armed}

    def force_disarm(self):
        """Force disarm the drone"""
        print("\n⚠️  Force disarming...")
        # Send disarm command multiple times
        for i in range(5):
            self.send_command(MSP_DISARM, b'')
            time.sleep(0.1)
        print("✓ Disarm command sent")

    def set_motors(self, values):
        motors = values[:4] + [1000] * 4
        payload = b''
        for m in motors:
            payload += struct.pack('<H', m)
        self.send_command(MSP_SET_MOTOR, payload)

    def get_motors(self):
        payload = self.send_command(MSP_MOTOR)
        if not payload or len(payload) < 16:
            return None
        motors = []
        for i in range(4):
            motors.append(struct.unpack('<H', payload[i*2:(i+1)*2])[0])
        return motors

    def test_sequence(self):
        """Run motor test sequence"""
        print("\n" + "="*60)
        print("  MOTOR TEST - MSP_SET_MOTOR (DISARMED MODE)")
        print("="*60)
        print("\nIMPORTANT: MSP_SET_MOTOR only works when DISARMED!")
        print()

        if not self.connect():
            return

        # Check if armed and auto-disarm
        status = self.get_status()
        if status and status['armed']:
            print("⚠️  WARNING: Drone is currently ARMED")
            choice = input("Do you want to force DISARM? (yes/no): ").strip().lower()
            if choice == 'yes':
                self.force_disarm()
                time.sleep(1)
                status = self.get_status()
                if status and status['armed']:
                    print("✗ Failed to disarm. Please disarm manually.")
                    self.disconnect()
                    return
            else:
                print("✗ Cannot test motors while ARMED. Exiting.")
                self.disconnect()
                return

        print("✓ Drone is DISARMED - ready for motor test\n")

        try:
            # Test 1: All motors off
            print("Test 1: Motors OFF (1000 PWM)")
            self.set_motors([1000, 1000, 1000, 1000])
            time.sleep(1)
            motors = self.get_motors()
            if motors:
                print(f"  MSP_MOTOR readback: {motors}")

            # Test 2: Gradual increase
            print("\nTest 2: Gradual increase (1000 → 2000)")
            for pwm in range(1000, 2000, 20):
                self.set_motors([pwm, pwm, pwm, pwm])
                print(f"  Setting: {pwm}", end='\r')
                time.sleep(0.1)
            print()

            motors = self.get_motors()
            if motors:
                print(f"  MSP_MOTOR readback: {motors}")

            # Test 3: Hold at 2000
            print("\nTest 3: Hold at 2000 PWM (3 seconds)")
            for i in range(30):
                self.set_motors([2000, 2000, 2000, 2000])
                time.sleep(0.1)

            motors = self.get_motors()
            if motors:
                print(f"  MSP_MOTOR readback: {motors}")

            # Test 4: Individual motors
            print("\nTest 4: Individual motor test")
            for i in range(4):
                print(f"  Motor {i+1}: 1300 PWM")
                values = [1000, 1000, 1000, 1000]
                values[i] = 1300
                self.set_motors(values)
                time.sleep(1)
                motors = self.get_motors()
                if motors:
                    print(f"    Readback: {motors}")

            # Test 5: Ramp down
            print("\nTest 5: Ramp down to off")
            for pwm in range(1300, 1000, -20):
                self.set_motors([pwm, pwm, pwm, pwm])
                print(f"  Setting: {pwm}", end='\r')
                time.sleep(0.1)
            print()

            # Final: Motors off
            print("\nFinal: Motors OFF")
            for i in range(10):
                self.set_motors([1000, 1000, 1000, 1000])
                time.sleep(0.1)

            print("\n✓ Test sequence complete!")
            print("\nNOTE:")
            print("  - If MSP_MOTOR shows correct values but Gazebo motors don't spin,")
            print("    the issue is in the UDP output (port 9002) or Gazebo plugin")
            print("  - Check SITL console for '[SITL] Sending PWM:' messages")
            print("  - Values should be non-zero when motors are commanded")

        except KeyboardInterrupt:
            print("\n\n✗ Test interrupted")
        finally:
            print("\nShutting down motors...")
            for i in range(10):
                self.set_motors([1000, 1000, 1000, 1000])
                time.sleep(0.05)
            self.disconnect()

def main():
    tester = MotorTester()
    tester.test_sequence()

if __name__ == '__main__':
    main()
