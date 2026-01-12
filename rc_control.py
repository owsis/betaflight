#!/usr/bin/env python3
"""
Betaflight SITL - RC Controller dengan Keyboard Control
Script untuk mengirim perintah RC ke SITL dan mengontrol motor
Menggunakan keyboard sebagai virtual remote untuk mengatasi RX_FAILSAFE
"""

import socket
import struct
import time
import sys
import threading
from collections import defaultdict

# MSP Protocol Constants
MSP_HEADER = b'$M<'
MSP_SET_RAW_RC = 200
MSP_STATUS = 101
MSP_MOTOR = 104

# RC Channel defaults (1000-2000, center 1500)
RC_MIN = 1000
RC_CENTER = 1500
RC_MAX = 2000

class BetaflightRC:
    def __init__(self, host='127.0.0.1', port=5762):  # Port 5762
        self.host = host
        self.port = port
        self.sock = None

        # RC Channels (AETR1234 - 8 channels)
        # 0: Roll, 1: Pitch, 2: Throttle, 3: Yaw
        # 4-7: AUX1-4
        self.channels = [RC_CENTER] * 8
        self.channels[2] = RC_MIN  # Throttle starts at minimum

        # State for keyboard control
        self.running = False
        self.armed = False
        self.keys_pressed = defaultdict(bool)

        # Thread safety
        self.sock_lock = threading.Lock()
        self.connected = False

    def connect(self):
        """Connect to Betaflight SITL"""
        try:
            with self.sock_lock:
                if self.sock:
                    try:
                        self.sock.close()
                    except:
                        pass

                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                self.sock.connect((self.host, self.port))
                self.connected = True
                print(f"✓ Connected to Betaflight SITL at {self.host}:{self.port}")
            return True
        except Exception as e:
            self.connected = False
            print(f"✗ Connection failed: {e}")
            return False

    def disconnect(self):
        """Disconnect from SITL"""
        with self.sock_lock:
            self.connected = False
            if self.sock:
                try:
                    self.sock.close()
                except:
                    pass
                self.sock = None
                print("✓ Disconnected")

    def msp_checksum(self, data):
        """Calculate MSP checksum"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def send_msp_command(self, cmd, payload=b''):
        """Send MSP command with error handling"""
        if not self.connected:
            return False

        try:
            size = len(payload)
            packet = MSP_HEADER + bytes([size, cmd]) + payload
            checksum = self.msp_checksum(bytes([size, cmd]) + payload)
            packet += bytes([checksum])

            with self.sock_lock:
                if self.sock:
                    self.sock.send(packet)
                    return True
                else:
                    return False
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            self.connected = False
            print(f"\n⚠️  Connection lost: {e}")
            return False
        except Exception as e:
            print(f"\n✗ Send error: {e}")
            return False

    def set_rc(self, roll=None, pitch=None, throttle=None, yaw=None,
               aux1=None, aux2=None, aux3=None, aux4=None, send_immediate=None):
        """
        Set RC channel values

        Args:
            roll: -100 to 100 (left/right)
            pitch: -100 to 100 (forward/back)
            throttle: 0 to 100 (up)
            yaw: -100 to 100 (rotate left/right)
            aux1-4: 0 or 1 (switch off/on)
            send_immediate: If True, send immediately. If False, only update channels.
                          If None, send only if RC loop is not running.
        """
        # Update channels if provided
        if roll is not None:
            self.channels[0] = int(RC_CENTER + (roll * 5))  # -500 to +500
        if pitch is not None:
            self.channels[1] = int(RC_CENTER + (pitch * 5))
        if throttle is not None:
            self.channels[2] = int(RC_MIN + (throttle * 10))  # 1000 to 2000
        if yaw is not None:
            self.channels[3] = int(RC_CENTER + (yaw * 5))

        # AUX switches (1000=OFF, 2000=ON)
        if aux1 is not None:
            self.channels[4] = RC_MAX if aux1 else RC_MIN
        if aux2 is not None:
            self.channels[5] = RC_MAX if aux2 else RC_MIN
        if aux3 is not None:
            self.channels[6] = RC_MAX if aux3 else RC_MIN
        if aux4 is not None:
            self.channels[7] = RC_MAX if aux4 else RC_MIN

        # Constrain values
        for i in range(len(self.channels)):
            self.channels[i] = max(RC_MIN, min(RC_MAX, self.channels[i]))

        # Decide whether to send immediately
        if send_immediate is None:
            # If RC loop is running, don't send (loop will handle it)
            send_immediate = not self.running

        # Send MSP_SET_RAW_RC if requested
        if send_immediate:
            payload = struct.pack('<' + 'H' * 8, *self.channels)
            self.send_msp_command(MSP_SET_RAW_RC, payload)

    def arm(self):
        """ARM the drone (enable motors) - via AUX1 switch"""
        print("\n⚠️  ARMING DRONE via AUX1 switch...")

        # Set AUX1 HIGH (2000) to ARM
        # Make sure throttle is LOW
        self.set_rc(throttle=0, aux1=1)

        print("✓ AUX1 set to HIGH (ARM)")
        print("   Drone should ARM now!")

    def disarm(self):
        """DISARM the drone (disable motors) - via AUX1 switch"""
        print("\n⚠️  DISARMING DRONE via AUX1 switch...")

        # Set AUX1 LOW (1000) to DISARM
        # Set throttle to LOW
        self.set_rc(throttle=0, aux1=0)

        print("✓ AUX1 set to LOW (DISARM)")
        print("   Drone should DISARM now!")

    def hover(self, throttle=50, duration=5):
        """Hover at specified throttle level"""
        print(f"\n🚁 HOVERING at {throttle}% throttle for {duration} seconds...")

        end_time = time.time() + duration
        while time.time() < end_time:
            self.set_rc(throttle=throttle, roll=0, pitch=0, yaw=0)
            remaining = int(end_time - time.time())
            print(f"   Hovering... {remaining}s remaining", end='\r')
            time.sleep(0.1)

        print("\n✓ Hover complete")

    def rc_loop_thread(self):
        """Thread untuk mengirim RC commands secara kontinyu (10Hz)"""
        rate = 0.1  # 10Hz
        consecutive_failures = 0
        max_failures = 10

        while self.running:
            try:
                payload = struct.pack('<' + 'H' * 8, *self.channels)
                success = self.send_msp_command(MSP_SET_RAW_RC, payload)

                if success:
                    consecutive_failures = 0
                else:
                    consecutive_failures += 1
                    if consecutive_failures >= max_failures:
                        print(f"\n✗ Too many failures ({consecutive_failures}), stopping RC loop")
                        self.running = False
                        break

                time.sleep(rate)
            except Exception as e:
                print(f"\n✗ RC loop error: {e}")
                break

    def keyboard_control(self):
        """Manual control menggunakan keyboard"""
        print("\n" + "="*50)
        print("  KEYBOARD CONTROL MODE")
        print("="*50)
        print("\nKeyboard Controls:")
        print("  W/S: Pitch (forward/back)")
        print("  A/D: Roll (left/right)")
        print("  Q/E: Yaw (rotate left/right)")
        print("  I/K: Throttle (up/down) - 10% steps")
        print("  U/J: Throttle (up/down) - 1% steps")
        print("  R: ARM")
        print("  F: DISARM")
        print("  SPACE: Reset center (Roll/Pitch/Yaw to neutral)")
        print("  ESC atau Ctrl+C: Quit")
        print("\n" + "="*50)

        # Check if pynput available
        try:
            from pynput import keyboard
            use_pynput = True
            print("✓ Using pynput for keyboard input")
        except ImportError:
            use_pynput = False
            print("⚠️  pynput not found, using basic input mode")
            print("   Install with: pip3 install pynput")

        # Start RC loop thread
        self.running = True
        rc_thread = threading.Thread(target=self.rc_loop_thread, daemon=True)
        rc_thread.start()

        print("\n📡 Starting RC signal (prevents RX_FAILSAFE)...")
        time.sleep(1)
        print("✓ RC signal active at 10Hz")

        if use_pynput:
            self._keyboard_control_pynput()
        else:
            self._keyboard_control_basic()

        self.running = False

    def _keyboard_control_pynput(self):
        """Keyboard control menggunakan pynput library"""
        from pynput import keyboard

        # Throttle control variables
        throttle_percent = 0
        roll_percent = 0
        pitch_percent = 0
        yaw_percent = 0

        def update_display():
            """Update tampilan status"""
            armed_status = "🟢 ARMED" if self.armed else "🔴 DISARMED"
            print(f"\r{armed_status} | Throttle: {throttle_percent:3}% | Roll: {roll_percent:+4}% | Pitch: {pitch_percent:+4}% | Yaw: {yaw_percent:+4}%", end='', flush=True)

        def on_press(key):
            nonlocal throttle_percent, roll_percent, pitch_percent, yaw_percent

            try:
                # Check connection
                if not self.connected:
                    print("\n✗ Lost connection to SITL!")
                    return False

                # Throttle control - Coarse (10%)
                if key == keyboard.Key.up or (hasattr(key, 'char') and key.char == 'i'):
                    throttle_percent = min(100, throttle_percent + 10)
                    self.set_rc(throttle=throttle_percent)
                elif key == keyboard.Key.down or (hasattr(key, 'char') and key.char == 'k'):
                    throttle_percent = max(0, throttle_percent - 10)
                    self.set_rc(throttle=throttle_percent)

                # Throttle control - Fine (1%)
                elif hasattr(key, 'char') and key.char == 'u':
                    throttle_percent = min(100, throttle_percent + 1)
                    self.set_rc(throttle=throttle_percent)
                elif hasattr(key, 'char') and key.char == 'j':
                    throttle_percent = max(0, throttle_percent - 1)
                    self.set_rc(throttle=throttle_percent)

                # Pitch (Forward/Backward)
                elif hasattr(key, 'char') and key.char == 'w':
                    pitch_percent = min(100, pitch_percent + 10)
                    self.set_rc(pitch=pitch_percent)
                elif hasattr(key, 'char') and key.char == 's':
                    pitch_percent = max(-100, pitch_percent - 10)
                    self.set_rc(pitch=pitch_percent)

                # Roll (Left/Right)
                elif hasattr(key, 'char') and key.char == 'a':
                    roll_percent = max(-100, roll_percent - 10)
                    self.set_rc(roll=roll_percent)
                elif hasattr(key, 'char') and key.char == 'd':
                    roll_percent = min(100, roll_percent + 10)
                    self.set_rc(roll=roll_percent)

                # Yaw (Rotate)
                elif hasattr(key, 'char') and key.char == 'q':
                    yaw_percent = max(-100, yaw_percent - 10)
                    self.set_rc(yaw=yaw_percent)
                elif hasattr(key, 'char') and key.char == 'e':
                    yaw_percent = min(100, yaw_percent + 10)
                    self.set_rc(yaw=yaw_percent)

                # Reset center
                elif key == keyboard.Key.space:
                    roll_percent = 0
                    pitch_percent = 0
                    yaw_percent = 0
                    self.set_rc(roll=0, pitch=0, yaw=0)
                    print("\n✓ Reset to center (Roll/Pitch/Yaw = 0)")

                # ARM
                elif hasattr(key, 'char') and key.char == 'r':
                    print("\n⚙️  Arming...")
                    self.arm()
                    self.armed = True
                    print("✓ ARMED!")

                # DISARM
                elif hasattr(key, 'char') and key.char == 'f':
                    print("\n⚙️  Disarming...")
                    self.disarm()
                    self.armed = False
                    throttle_percent = 0
                    print("✓ DISARMED!")

                # Quit
                elif key == keyboard.Key.esc:
                    print("\n\n⚠️  Quitting...")
                    return False

                update_display()

            except AttributeError:
                pass

        print("\n✓ Keyboard control active!")
        print("Press ESC to quit\n")
        update_display()

        # Listen to keyboard
        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

    def _keyboard_control_basic(self):
        """Basic keyboard control tanpa pynput"""
        print("\n⚠️  Basic mode: Type commands and press ENTER")
        print("\nCommands:")
        print("  t <value>  - Set throttle (0-100)")
        print("  r <value>  - Set roll (-100 to 100)")
        print("  p <value>  - Set pitch (-100 to 100)")
        print("  y <value>  - Set yaw (-100 to 100)")
        print("  arm        - ARM motors")
        print("  disarm     - DISARM motors")
        print("  center     - Reset to center")
        print("  quit       - Exit")

        while self.running:
            try:
                cmd = input("\n> ").strip().lower().split()
                if not cmd:
                    continue

                if cmd[0] == 'quit' or cmd[0] == 'q':
                    break
                elif cmd[0] == 'arm':
                    self.arm()
                    self.armed = True
                elif cmd[0] == 'disarm':
                    self.disarm()
                    self.armed = False
                elif cmd[0] == 'center':
                    self.set_rc(roll=0, pitch=0, yaw=0)
                    print("✓ Reset to center")
                elif cmd[0] == 't' and len(cmd) > 1:
                    val = int(cmd[1])
                    self.set_rc(throttle=val)
                    print(f"✓ Throttle: {val}%")
                elif cmd[0] == 'r' and len(cmd) > 1:
                    val = int(cmd[1])
                    self.set_rc(roll=val)
                    print(f"✓ Roll: {val}%")
                elif cmd[0] == 'p' and len(cmd) > 1:
                    val = int(cmd[1])
                    self.set_rc(pitch=val)
                    print(f"✓ Pitch: {val}%")
                elif cmd[0] == 'y' and len(cmd) > 1:
                    val = int(cmd[1])
                    self.set_rc(yaw=val)
                    print(f"✓ Yaw: {val}%")
                else:
                    print("✗ Unknown command")

            except (ValueError, KeyboardInterrupt):
                break

    def test_flight(self):
        """Run a simple test flight sequence"""
        print("\n" + "="*50)
        print("  BETAFLIGHT SITL - TEST FLIGHT SEQUENCE")
        print("="*50)

        try:
            # Keep sending RC to maintain connection
            print("\n1. Initializing RC connection...")
            for i in range(10):
                self.set_rc(throttle=0)
                time.sleep(0.1)

            input("\nPress ENTER to ARM the drone...")
            self.arm()
            time.sleep(2)

            input("\nPress ENTER to TAKEOFF (increase throttle)...")
            print("🚁 Taking off...")

            # Gradual throttle increase
            for throttle in range(0, 55, 5):
                self.set_rc(throttle=throttle)
                print(f"   Throttle: {throttle}%", end='\r')
                time.sleep(0.5)
            print()

            # Hover
            self.hover(throttle=55, duration=5)

            input("\nPress ENTER to LAND (decrease throttle)...")
            print("🛬 Landing...")

            # Gradual throttle decrease
            for throttle in range(55, -1, -5):
                self.set_rc(throttle=throttle)
                print(f"   Throttle: {throttle}%", end='\r')
                time.sleep(0.5)
            print()

            time.sleep(2)

            input("\nPress ENTER to DISARM...")
            self.disarm()

            print("\n✓ Test flight complete!")

        except KeyboardInterrupt:
            print("\n\n⚠️  Flight interrupted!")
            print("Landing and disarming...")
            self.set_rc(throttle=0)
            time.sleep(1)
            self.disarm()

def main():
    print("="*50)
    print("  Betaflight SITL - RC Controller")
    print("="*50)
    print("\nPastikan SITL dan Gazebo sudah running!")
    print()

    # Create RC controller
    rc = BetaflightRC()

    # Connect
    if not rc.connect():
        print("\n✗ Tidak bisa connect. Pastikan SITL sudah running!")
        print("  Jalankan: ./run_sitl.sh")
        return

    print("\nMode:")
    print("  1. Test Flight (Otomatis)")
    print("  2. Manual Control (Keyboard)")
    print("  3. ARM Only")
    print()

    try:
        choice = input("Pilih mode (1/2/3): ").strip()

        if choice == "1":
            rc.test_flight()

        elif choice == "2":
            rc.keyboard_control()

        elif choice == "3":
            print("\n⚠️  ARM mode selected")
            input("Press ENTER to ARM...")
            rc.arm()
            print("\n✓ Drone ARMED!")
            print("Use Betaflight Configurator or another script to control")
            input("\nPress ENTER to DISARM...")
            rc.disarm()

        else:
            print("Invalid choice")

    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")

    finally:
        # Cleanup
        rc.set_rc(throttle=0)
        time.sleep(0.5)
        rc.disconnect()

if __name__ == "__main__":
    main()
