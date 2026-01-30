#!/usr/bin/env python3
"""
Test connection to drone - supports both SITL and real hardware.

Usage:
    SITL:  python3 test_connection.py --sitl
    Real:  python3 test_connection.py --serial /dev/ttyAMA0
"""

import asyncio
import argparse
from mavsdk import System

SITL_CONNECTION = "udp://:14540"
SERIAL_BAUD = 57600


def parse_args():
    parser = argparse.ArgumentParser(description="Test drone connection")
    conn_group = parser.add_mutually_exclusive_group(required=True)
    conn_group.add_argument("--sitl", action="store_true", help="Connect to SITL")
    conn_group.add_argument("--serial", type=str, help="Serial port (e.g., /dev/ttyAMA0)")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD, help="Baud rate")
    return parser.parse_args()


async def run(args):
    if args.sitl:
        conn = SITL_CONNECTION
    else:
        conn = f"serial://{args.serial}:{args.baud}"
    
    print(f"Connecting to: {conn}")
    
    drone = System()
    await drone.connect(system_address=conn)
    
    print("Waiting for drone...", end=" ", flush=True)
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Drone discovered!")
            break
    
    # Get basic telemetry
    print("\n📊 Telemetry:")
    
    async for battery in drone.telemetry.battery():
        print(f"  Battery: {battery.remaining_percent * 100:.1f}%")
        break
    
    async for gps in drone.telemetry.gps_info():
        print(f"  GPS: {gps.num_satellites} satellites, fix: {gps.fix_type}")
        break
    
    async for position in drone.telemetry.position():
        print(f"  Position: ({position.latitude_deg:.6f}, {position.longitude_deg:.6f})")
        print(f"  Altitude: {position.relative_altitude_m:.1f}m AGL")
        break
    
    async for health in drone.telemetry.health():
        print(f"  Armable: {'✅ Yes' if health.is_armable else '❌ No'}")
        break
    
    print("\n✅ Connection test complete!")


if __name__ == "__main__":
    args = parse_args()
    asyncio.run(run(args))