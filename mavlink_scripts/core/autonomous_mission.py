#!/usr/bin/env python3
"""
Autonomous Mission for BLE Tag Scanning Drone
================================================
Supports both SITL simulation and real hardware (RPi5 + Pixhawk 6C).

Usage:
    SITL:     python3 autonomous_mission.py --sitl
    Real:     python3 autonomous_mission.py --serial /dev/ttyAMA0
    Real:     python3 autonomous_mission.py --serial /dev/ttyUSB0 --baud 921600
"""

import asyncio
import argparse
import math
import sys
from datetime import datetime
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

# --- Configuration ---
DEFAULT_FLY_ALTITUDE = 15.0      # Meters AGL
DEFAULT_FLY_SPEED = 2.0          # m/s
DEFAULT_GRID_SPACING = 10.0      # Distance between scan lines (meters)
DEFAULT_AREA_WIDTH = 40.0        # East-West coverage (meters)
DEFAULT_AREA_HEIGHT = 50.0       # North-South coverage (meters)
DEFAULT_NUM_LINES = 5            # Number of parallel scan lines

# Safety thresholds
MIN_BATTERY_PERCENT = 30.0       # Don't start mission below this
MIN_SATELLITES = 8               # GPS satellites required for Position mode

# Connection defaults
SITL_CONNECTION = "udp://:14540"
SERIAL_BAUD = 57600              # Default MAVLink baud rate


def parse_args():
    parser = argparse.ArgumentParser(
        description="Autonomous grid mission for BLE tag scanning drone"
    )
    
    # Connection options (mutually exclusive)
    conn_group = parser.add_mutually_exclusive_group(required=True)
    conn_group.add_argument(
        "--sitl", action="store_true",
        help="Connect to SITL simulation (udp://:14540)"
    )
    conn_group.add_argument(
        "--serial", type=str, metavar="PORT",
        help="Serial port for real hardware (e.g., /dev/ttyAMA0, /dev/ttyUSB0)"
    )
    
    # Serial options
    parser.add_argument(
        "--baud", type=int, default=SERIAL_BAUD,
        help=f"Serial baud rate (default: {SERIAL_BAUD})"
    )
    
    # Mission parameters
    parser.add_argument(
        "--altitude", type=float, default=DEFAULT_FLY_ALTITUDE,
        help=f"Flight altitude in meters AGL (default: {DEFAULT_FLY_ALTITUDE})"
    )
    parser.add_argument(
        "--speed", type=float, default=DEFAULT_FLY_SPEED,
        help=f"Flight speed in m/s (default: {DEFAULT_FLY_SPEED})"
    )
    parser.add_argument(
        "--width", type=float, default=DEFAULT_AREA_WIDTH,
        help=f"Scan area width in meters (default: {DEFAULT_AREA_WIDTH})"
    )
    parser.add_argument(
        "--height", type=float, default=DEFAULT_AREA_HEIGHT,
        help=f"Scan area height in meters (default: {DEFAULT_AREA_HEIGHT})"
    )
    parser.add_argument(
        "--spacing", type=float, default=DEFAULT_GRID_SPACING,
        help=f"Grid line spacing in meters (default: {DEFAULT_GRID_SPACING})"
    )
    
    # Safety overrides
    parser.add_argument(
        "--skip-checks", action="store_true",
        help="Skip pre-flight safety checks (USE WITH CAUTION)"
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Upload mission but don't arm or takeoff"
    )
    
    return parser.parse_args()


def get_connection_string(args) -> str:
    """Build the MAVSDK connection string."""
    if args.sitl:
        return SITL_CONNECTION
    else:
        # Serial connection format: serial:///dev/ttyXXX:BAUD
        return f"serial://{args.serial}:{args.baud}"


def generate_grid_waypoints(lat: float, lon: float, config) -> list:
    """
    Generate a lawnmower/zigzag grid pattern for scanning.
    
    The pattern starts from the drone's current position and extends:
    - North (latitude+) for 'height' meters
    - East (longitude+) for 'width' meters
    """
    waypoints = []
    
    altitude = config['altitude']
    speed = config['speed']
    area_width = config['width']
    area_height = config['height']
    spacing = config['spacing']
    
    # Convert meters to degrees
    # 1 degree latitude ≈ 111,111 meters
    # 1 degree longitude ≈ 111,111 * cos(latitude) meters
    lat_per_meter = 1.0 / 111111.0
    lon_per_meter = 1.0 / (111111.0 * math.cos(math.radians(lat)))
    
    lat_extent = area_height * lat_per_meter
    lon_extent = area_width * lon_per_meter
    lat_step = spacing * lat_per_meter
    
    # Grid boundaries
    lat_min = lat
    lat_max = lat + lat_extent
    lon_min = lon
    lon_max = lon + lon_extent
    
    current_lat = lat_min
    going_east = True
    line_count = 0
    
    print(f"  Grid: {area_width}m × {area_height}m, spacing {spacing}m")
    print(f"  Start: ({lat:.6f}, {lon:.6f})")
    print(f"  End:   ({lat_max:.6f}, {lon_max:.6f})")
    
    while current_lat <= lat_max:
        # Define line start and end points
        if going_east:
            p1 = (current_lat, lon_min)
            p2 = (current_lat, lon_max)
        else:
            p1 = (current_lat, lon_max)
            p2 = (current_lat, lon_min)
        
        # Add waypoints for this line
        for point in [p1, p2]:
            waypoints.append(MissionItem(
                point[0],                           # latitude_deg
                point[1],                           # longitude_deg
                altitude,                           # relative_altitude_m
                speed,                              # speed_m_s
                True,                               # is_fly_through
                float('nan'),                       # gimbal_pitch_deg
                float('nan'),                       # gimbal_yaw_deg
                MissionItem.CameraAction.NONE,      # camera_action
                float('nan'),                       # loiter_time_s
                float('nan'),                       # camera_photo_interval_s
                float('nan'),                       # acceptance_radius_m
                float('nan'),                       # yaw_deg
                float('nan'),                       # camera_photo_distance_m
                MissionItem.VehicleAction.NONE      # vehicle_action
            ))
        
        current_lat += lat_step
        going_east = not going_east
        line_count += 1
    
    print(f"  Generated {len(waypoints)} waypoints across {line_count} scan lines")
    return waypoints


async def preflight_checks(drone: System, skip_checks: bool = False) -> bool:
    """
    Perform pre-flight safety checks.
    Returns True if safe to proceed, False otherwise.
    """
    if skip_checks:
        print("⚠️  Pre-flight checks SKIPPED (--skip-checks flag)")
        return True
    
    print("\n🔍 Pre-flight Safety Checks")
    print("-" * 40)
    all_passed = True
    
    # Check 1: Battery level
    print("  [1/4] Battery level...", end=" ", flush=True)
    try:
        async for battery in drone.telemetry.battery():
            percent = battery.remaining_percent * 100
            if percent < MIN_BATTERY_PERCENT:
                print(f"❌ FAIL ({percent:.1f}% < {MIN_BATTERY_PERCENT}%)")
                all_passed = False
            else:
                print(f"✅ OK ({percent:.1f}%)")
            break
    except Exception as e:
        print(f"⚠️  Unable to read ({e})")
    
    # Check 2: GPS fix
    print("  [2/4] GPS satellites...", end=" ", flush=True)
    try:
        async for gps_info in drone.telemetry.gps_info():
            sats = gps_info.num_satellites
            fix = gps_info.fix_type
            if sats < MIN_SATELLITES:
                print(f"❌ FAIL ({sats} sats < {MIN_SATELLITES} required)")
                all_passed = False
            else:
                print(f"✅ OK ({sats} sats, fix: {fix})")
            break
    except Exception as e:
        print(f"⚠️  Unable to read ({e})")
    
    # Check 3: Health checks
    print("  [3/4] System health...", end=" ", flush=True)
    try:
        async for health in drone.telemetry.health():
            issues = []
            # Check available health attributes
            if hasattr(health, 'is_accelerometer_calibration_ok') and not health.is_accelerometer_calibration_ok:
                issues.append("accel")
            if hasattr(health, 'is_gyrometer_calibration_ok') and not health.is_gyrometer_calibration_ok:
                issues.append("gyro")
            if hasattr(health, 'is_magnetometer_calibration_ok') and not health.is_magnetometer_calibration_ok:
                issues.append("mag")
            if hasattr(health, 'is_home_position_ok') and not health.is_home_position_ok:
                issues.append("home")
            
            if issues:
                print(f"⚠️  Issues: {', '.join(issues)}")
            else:
                print("✅ OK")
            break
    except Exception as e:
        print(f"⚠️  Unable to read ({e})")
    
    # Check 4: Armable
    print("  [4/4] Armable...", end=" ", flush=True)
    try:
        async for health in drone.telemetry.health():
            if health.is_armable:
                print("✅ OK")
            else:
                print("❌ FAIL (not armable)")
                all_passed = False
            break
    except Exception as e:
        print(f"⚠️  Unable to read ({e})")
    
    print("-" * 40)
    if all_passed:
        print("✅ All pre-flight checks PASSED")
    else:
        print("❌ Pre-flight checks FAILED - mission aborted")
    
    return all_passed


async def run(args):
    """Main mission execution."""
    connection_string = get_connection_string(args)
    
    print("=" * 50)
    print("  BLE Tag Scanner - Autonomous Mission")
    print("=" * 50)
    print(f"  Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"  Mode: {'SITL Simulation' if args.sitl else 'REAL HARDWARE'}")
    print(f"  Connection: {connection_string}")
    print("=" * 50)
    
    if not args.sitl:
        print("\n⚠️  WARNING: REAL HARDWARE MODE")
        print("    Ensure props are clear and area is safe!")
        await asyncio.sleep(2)
    
    # Connect to drone
    drone = System()
    print(f"\n📡 Connecting to drone...")
    await drone.connect(system_address=connection_string)
    
    print("  Waiting for connection...", end=" ", flush=True)
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Connected!")
            break
    
    # Get initial position
    print("  Getting GPS position...", end=" ", flush=True)
    async for position in drone.telemetry.position():
        takeoff_lat = position.latitude_deg
        takeoff_lon = position.longitude_deg
        takeoff_alt = position.absolute_altitude_m
        print(f"✅ ({takeoff_lat:.6f}, {takeoff_lon:.6f})")
        break
    
    # Pre-flight checks
    if not await preflight_checks(drone, args.skip_checks):
        print("\n🛑 Mission aborted due to failed pre-flight checks")
        return 1
    
    # Generate mission
    print("\n📍 Generating Mission Waypoints")
    config = {
        'altitude': args.altitude,
        'speed': args.speed,
        'width': args.width,
        'height': args.height,
        'spacing': args.spacing,
    }
    mission_items = generate_grid_waypoints(takeoff_lat, takeoff_lon, config)
    mission_plan = MissionPlan(mission_items)
    
    # Upload mission
    print("\n📤 Uploading Mission...")
    await drone.mission.set_return_to_launch_after_mission(True)
    await drone.mission.upload_mission(mission_plan)
    print("  ✅ Mission uploaded")
    
    # Dry run check
    if args.dry_run:
        print("\n🔶 DRY RUN - Mission uploaded but NOT executing")
        print("    Remove --dry-run flag to execute mission")
        return 0
    
    # Arm and takeoff
    print("\n🚁 Arming...")
    await drone.action.arm()
    print("  ✅ Armed")
    
    print("  Taking off...", end=" ", flush=True)
    await drone.action.takeoff()
    print("✅")
    
    # Wait for takeoff completion
    print("  Climbing to mission altitude...", end=" ", flush=True)
    await asyncio.sleep(10)  # Allow time to reach altitude
    print("✅")
    
    # Start mission
    print("\n▶️  Starting Mission...")
    await drone.mission.start_mission()
    
    # Monitor mission progress
    print("\n📊 Mission Progress")
    print("-" * 30)
    
    async for mission_progress in drone.mission.mission_progress():
        current = mission_progress.current
        total = mission_progress.total
        percent = (current / total * 100) if total > 0 else 0
        bar = "█" * int(percent // 5) + "░" * (20 - int(percent // 5))
        print(f"\r  [{bar}] {current}/{total} ({percent:.0f}%)", end="", flush=True)
        
        if current == total and current > 0:
            print()  # Newline
            break
    
    print("-" * 30)
    print("✅ Mission Complete!")
    
    # RTL is automatic due to set_return_to_launch_after_mission(True)
    print("\n🏠 Returning to Launch...")
    
    # Monitor landing
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break
    
    print("✅ Landed safely!")
    print("\n" + "=" * 50)
    print("  Mission completed successfully!")
    print("=" * 50)
    
    return 0


if __name__ == "__main__":
    args = parse_args()
    try:
        exit_code = asyncio.run(run(args))
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\n\n⚠️  Mission interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Error: {e}")
        sys.exit(1)