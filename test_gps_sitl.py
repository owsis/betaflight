#!/usr/bin/env python3
"""
Test script to verify GPS SITL is receiving data
"""
import socket
import struct
import time

# FDM packet structure (must match betaflight target.h fdm_packet)
def create_fdm_packet():
    """Create a test FDM packet with GPS data"""
    # Calculate actual struct size:
    # timestamp(8) + ang_vel[3](24) + lin_acc[3](24) + quat[4](32) +
    # velocity[3](24) + position[3](24) + pressure(8) +
    # lat(8) + lon(8) + alt_msl(8) + gps_vel[3](24) + num_sat(1) + fix_type(1)
    # = 194 bytes, round up to 200 for alignment
    packet = bytearray(200)
    
    # Magic header (not in struct, but useful for debugging)
    # struct.pack_into('<I', packet, 0, 0x4c475a47)  # 'GZGL' - skip this
    
    # Timestamp
    struct.pack_into('<d', packet, 0, time.time())
    
    # IMU angular velocity (rad/s) - roll, pitch, yaw rates
    offset = 8
    for i in range(3):  # angular velocity
        struct.pack_into('<d', packet, offset + i*8, 0.0)
    
    # IMU linear acceleration (m/s²)
    offset = 32
    for i in range(3):  # linear acceleration
        struct.pack_into('<d', packet, offset + i*8, 0.0 if i < 2 else -9.80665)  # gravity
    
    # IMU orientation quaternion (w, x, y, z) - identity
    offset = 56
    struct.pack_into('<d', packet, offset, 1.0)    # w
    struct.pack_into('<d', packet, offset+8, 0.0)  # x
    struct.pack_into('<d', packet, offset+16, 0.0) # y
    struct.pack_into('<d', packet, offset+24, 0.0) # z
    
    # Velocity (m/s, earth frame NED)
    offset = 88
    struct.pack_into('<d', packet, offset, 1.0)    # 1 m/s North
    struct.pack_into('<d', packet, offset+8, 0.5)  # 0.5 m/s East
    struct.pack_into('<d', packet, offset+16, 0.0) # 0 m/s Down
    
    # Position (meters, NED from origin)
    offset = 112
    struct.pack_into('<d', packet, offset, 5.0)      # 5m North
    struct.pack_into('<d', packet, offset+8, 3.0)    # 3m East
    struct.pack_into('<d', packet, offset+16, -2.0)  # 2m Up (negative D)
    
    # Pressure (Pa)
    offset = 136
    struct.pack_into('<d', packet, offset, 101325.0)
    
    # GPS data
    offset = 144
    # Latitude: -35.3632607 degrees
    struct.pack_into('<d', packet, offset, -35.3632607)
    
    # Longitude: 149.1652351 degrees  
    struct.pack_into('<d', packet, offset+8, 149.1652351)
    
    # Altitude MSL: 584 meters
    struct.pack_into('<d', packet, offset+16, 584.0)
    
    # GPS velocity NED (m/s)
    struct.pack_into('<d', packet, offset+24, 1.0)    # North
    struct.pack_into('<d', packet, offset+32, 0.5)    # East
    struct.pack_into('<d', packet, offset+40, 0.0)    # Down
    
    # Number of satellites
    struct.pack_into('<B', packet, offset+48, 12)
    
    # GPS fix type (3 = 3D fix)
    struct.pack_into('<B', packet, offset+49, 3)
    
    return bytes(packet)

def main():
    print("GPS SITL Test - Sending FDM packets with GPS data")
    print("=" * 60)
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # SITL listens on port 9003 for FDM data
    sitl_address = ('127.0.0.1', 9003)
    
    # Create packet template as bytearray (mutable)
    packet = bytearray(create_fdm_packet())
    
    print(f"Packet size: {len(packet)} bytes")
    print(f"Sending to: {sitl_address}")
    print("\nGPS data in packet:")
    print(f"  Latitude:  -35.3632607°")
    print(f"  Longitude: 149.1652351°")
    print(f"  Altitude:  584m MSL")
    print(f"  Satellites: 12")
    print(f"  Fix type:  3 (3D fix)")
    print("\nSending packets every 0.1 second...")
    print("Press Ctrl+C to stop\n")
    
    try:
        count = 0
        while True:
            # Update timestamp in packet
            struct.pack_into('<d', packet, 0, time.time())
            
            # Send packet
            sock.sendto(packet, sitl_address)
            count += 1
            
            if count % 10 == 0:
                print(f"Sent {count} packets...")
            
            time.sleep(0.1)  # 10 Hz
            
    except KeyboardInterrupt:
        print(f"\n\nSent {count} packets total")
        print("Test completed")
    finally:
        sock.close()

if __name__ == '__main__':
    main()
