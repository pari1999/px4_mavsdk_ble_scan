"""
Simulation Harness for BLE Scanner
Connects drone (MAVSDK) with BLE simulator for integrated testing.
"""

import asyncio
import argparse
from mavsdk import System

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from simulation.ble_simulator import BLESimulator


class SimulationHarness:
    """Manages drone and BLE simulation together."""
    
    def __init__(self, connection_url="udp://:14540"):
        self.connection_url = connection_url
        self.drone = None
        self.ble_simulator = BLESimulator()
        
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        
        self.discovered_tags = set()
        self.scan_count = 0
        self.is_polling = False
    
    async def connect(self):
        """Connect to drone."""
        print(f"📡 Connecting to {self.connection_url}...")
        self.drone = System()
        await self.drone.connect(system_address=self.connection_url)
        
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("✅ Connected!")
                return True
        return False
    
    async def load_tags(self, config_file="tags_config.yaml"):
        """Load tags from file or generate random ones."""
        try:
            self.ble_simulator.load_from_yaml(config_file)
            print(f"✅ Loaded {len(self.ble_simulator.tags)} tags")
        except FileNotFoundError:
            print("⚠️  Generating random tags...")
            async for pos in self.drone.telemetry.position():
                self.ble_simulator.generate_random_tags(
                    10, pos.latitude_deg, pos.longitude_deg, 30.0
                )
                break
            self.ble_simulator.save_to_yaml(config_file)
            print(f"✅ Generated {len(self.ble_simulator.tags)} tags")
    
    async def update_position(self):
        """Update drone position from telemetry."""
        async for pos in self.drone.telemetry.position():
            self.current_lat = pos.latitude_deg
            self.current_lon = pos.longitude_deg
            self.current_alt = pos.relative_altitude_m
            return
    
    async def scan_once(self):
        """Perform one RSSI scan."""
        await self.update_position()
        
        readings = self.ble_simulator.get_detected_tags(
            self.current_lat, self.current_lon, self.current_alt
        )
        self.scan_count += 1
        
        for reading in readings:
            tag_id = reading["tag_id"]
            if tag_id not in self.discovered_tags:
                self.discovered_tags.add(tag_id)
                print(f"🏷️  NEW: {tag_id} @ {reading['rssi']:.1f} dBm")
        
        return readings
    
    async def start_polling(self, poll_rate=10.0):
        """Start continuous RSSI polling."""
        self.is_polling = True
        poll_interval = 1.0 / poll_rate
        print(f"📶 Polling at {poll_rate} Hz")
        
        while self.is_polling:
            try:
                await self.scan_once()
                await asyncio.sleep(poll_interval)
            except asyncio.CancelledError:
                break
    
    def stop_polling(self):
        """Stop RSSI polling."""
        self.is_polling = False
    
    async def arm_and_takeoff(self, altitude=15.0):
        """Arm and take off."""
        print("\n🚁 Pre-flight...")
        
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok:
                print("   GPS ✅")
                break
        
        await self.update_position()
        print(f"   Position: ({self.current_lat:.6f}, {self.current_lon:.6f})")
        
        await self.drone.action.arm()
        print("   Armed ✅")
        
        await self.drone.action.set_takeoff_altitude(altitude)
        await self.drone.action.takeoff()
        await asyncio.sleep(8)
        print(f"   Takeoff to {altitude}m ✅")
    
    async def land(self):
        """Land the drone."""
        print("\n🛬 Landing...")
        await self.drone.action.land()
        
        async for armed in self.drone.telemetry.armed():
            if not armed:
                print("✅ Landed")
                break
    
    def print_statistics(self):
        """Print results."""
        total = len(self.ble_simulator.tags)
        found = len(self.discovered_tags)
        
        print("\n" + "=" * 40)
        print("📊 RESULTS")
        print(f"   Scans: {self.scan_count}")
        print(f"   Found: {found}/{total} ({found/total*100:.0f}%)" if total else "   No tags")
        print(f"   Tags: {sorted(self.discovered_tags)}")
        print("=" * 40)


async def run_test():
    """Run test mission."""
    print("\n" + "=" * 40)
    print("🧪 TEST MISSION")
    print("=" * 40)
    
    harness = SimulationHarness()
    await harness.connect()
    await harness.load_tags()
    await harness.arm_and_takeoff(15.0)
    
    polling_task = asyncio.create_task(harness.start_polling(10.0))
    
    print("\n⏳ Scanning for 15 seconds...")
    await asyncio.sleep(15)
    
    harness.stop_polling()
    polling_task.cancel()
    try:
        await polling_task
    except asyncio.CancelledError:
        pass
    
    await harness.land()
    harness.print_statistics()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", action="store_true", help="Run test mission")
    args = parser.parse_args()
    
    if args.test:
        asyncio.run(run_test())
    else:
        print("Usage: python3 simulation_harness.py --test")
