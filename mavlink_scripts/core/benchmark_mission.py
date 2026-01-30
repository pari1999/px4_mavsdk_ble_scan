"""
Benchmark: Lawnmower vs GP-based Exploration
=============================================
Compares simple lawnmower pattern against intelligent GP exploration.

Usage:
    python3 benchmark_mission.py --mode lawnmower
    python3 benchmark_mission.py --mode gp
    python3 benchmark_mission.py --mode both
"""

import asyncio
import time
import math
import argparse
from mavsdk import System

from ble_simulator import BLESimulator
from gaussian_process import AdaptivePlanner


# Configuration
SITL_CONNECTION = "udp://:14540"
FLIGHT_ALTITUDE = 15.0
MAX_MISSION_TIME = 300
WAYPOINT_ARRIVAL_THRESHOLD = 5.0
MAX_SPEED = 2.5


class MissionMetrics:
    """Track mission performance metrics."""
    
    def __init__(self):
        self.start_time = 0
        self.end_time = 0
        self.total_distance = 0
        self.tags_found = []
        self.discovery_times = {}  # tag_id -> time when found
        self.path = []  # [(lat, lon, time)]
        self.waypoints_visited = 0
    
    def start(self):
        self.start_time = time.time()
    
    def stop(self):
        self.end_time = time.time()
    
    def add_position(self, lat, lon):
        t = time.time() - self.start_time
        if self.path:
            # Calculate distance from last position
            last_lat, last_lon, _ = self.path[-1]
            R = 6371000
            dlat = math.radians(lat - last_lat)
            dlon = math.radians(lon - last_lon)
            a = math.sin(dlat/2)**2 + math.cos(math.radians(lat)) * \
                math.cos(math.radians(last_lat)) * math.sin(dlon/2)**2
            dist = R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
            self.total_distance += dist
        self.path.append((lat, lon, t))
    
    def tag_found(self, tag_id):
        if tag_id not in self.tags_found:
            self.tags_found.append(tag_id)
            self.discovery_times[tag_id] = time.time() - self.start_time
    
    def get_summary(self):
        duration = self.end_time - self.start_time
        return {
            "mode": "",
            "duration_s": round(duration, 1),
            "tags_found": len(self.tags_found),
            "distance_m": round(self.total_distance, 1),
            "waypoints": self.waypoints_visited,
            "efficiency": round(len(self.tags_found) / (self.total_distance / 100), 2) if self.total_distance > 0 else 0,
            "discovery_times": self.discovery_times
        }


class BaseMission:
    """Base class for mission types."""
    
    def __init__(self, connection_url=SITL_CONNECTION):
        self.connection_url = connection_url
        self.drone = None
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        self.absolute_altitude = 0.0
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.ble_simulator = BLESimulator()
        self.metrics = MissionMetrics()
        self.discovered_tags = set()
    
    async def connect(self):
        print(f"📡 Connecting...")
        self.drone = System()
        await self.drone.connect(system_address=self.connection_url)
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("✅ Connected!")
                return
    
    async def update_position(self):
        async for pos in self.drone.telemetry.position():
            self.current_lat = pos.latitude_deg
            self.current_lon = pos.longitude_deg
            self.current_alt = pos.relative_altitude_m
            self.absolute_altitude = pos.absolute_altitude_m
            return
    
    async def setup(self):
        await self.update_position()
        self.home_lat = self.current_lat
        self.home_lon = self.current_lon
        print(f"📍 Home: ({self.current_lat:.6f}, {self.current_lon:.6f})")
        
        self.ble_simulator.load_from_yaml("tags_config.yaml")
        print(f"✅ Loaded {len(self.ble_simulator.tags)} tags")
    
    async def arm_and_takeoff(self):
        print("🚁 Taking off...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok:
                break
        
        await self.drone.action.set_current_speed(MAX_SPEED)
        await self.drone.action.arm()
        await self.drone.action.set_takeoff_altitude(FLIGHT_ALTITUDE)
        await self.drone.action.takeoff()
        
        while True:
            await self.update_position()
            if self.current_alt >= FLIGHT_ALTITUDE * 0.9:
                break
            await asyncio.sleep(0.5)
        print(f"✅ At altitude: {self.current_alt:.1f}m")
    
    def scan(self):
        readings = self.ble_simulator.get_detected_tags(
            self.current_lat, self.current_lon, self.current_alt
        )
        for r in readings:
            tag_id = r["tag_id"]
            if tag_id not in self.discovered_tags:
                self.discovered_tags.add(tag_id)
                self.metrics.tag_found(tag_id)
                print(f"🏷️  FOUND: {tag_id}")
        return readings
    
    async def goto(self, lat, lon):
        target_amsl = self.absolute_altitude - self.current_alt + FLIGHT_ALTITUDE
        await self.drone.action.goto_location(lat, lon, target_amsl, 0)
        self.metrics.waypoints_visited += 1
    
    async def wait_arrival(self, target_lat, target_lon, timeout=60):
        start = time.time()
        while time.time() - start < timeout:
            await self.update_position()
            self.metrics.add_position(self.current_lat, self.current_lon)
            self.scan()
            
            dist = self.get_distance(target_lat, target_lon)
            if dist < WAYPOINT_ARRIVAL_THRESHOLD:
                return True
            
            if len(self.discovered_tags) >= len(self.ble_simulator.tags):
                return True
            
            await asyncio.sleep(0.5)
        return False
    
    def get_distance(self, lat, lon):
        R = 6371000
        dlat = math.radians(lat - self.current_lat)
        dlon = math.radians(lon - self.current_lon)
        a = math.sin(dlat/2)**2 + math.cos(math.radians(self.current_lat)) * \
            math.cos(math.radians(lat)) * math.sin(dlon/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    async def return_home(self):
        print("🏠 Returning...")
        await self.drone.action.return_to_launch()
        async for in_air in self.drone.telemetry.in_air():
            if not in_air:
                break
        print("✅ Landed!")


class LawnmowerMission(BaseMission):
    """Simple lawnmower pattern coverage."""
    
    def generate_lawnmower_waypoints(self, size=70, spacing=20):
        """Generate zigzag lawnmower pattern."""
        waypoints = []
        R = 6371000
        
        # Generate grid lines
        lines = int(size * 2 / spacing) + 1
        going_east = True
        
        for i in range(lines):
            y_offset = -size + i * spacing
            lat = self.home_lat + math.degrees(y_offset / R)
            
            if going_east:
                lon_start = self.home_lon + math.degrees(-size / (R * math.cos(math.radians(self.home_lat))))
                lon_end = self.home_lon + math.degrees(size / (R * math.cos(math.radians(self.home_lat))))
            else:
                lon_start = self.home_lon + math.degrees(size / (R * math.cos(math.radians(self.home_lat))))
                lon_end = self.home_lon + math.degrees(-size / (R * math.cos(math.radians(self.home_lat))))
            
            waypoints.append((lat, lon_start))
            waypoints.append((lat, lon_end))
            going_east = not going_east
        
        return waypoints
    
    async def run(self):
        print("\n" + "=" * 50)
        print("🔲 LAWNMOWER PATTERN MISSION")
        print("=" * 50)
        
        await self.connect()
        await self.setup()
        await self.arm_and_takeoff()
        
        waypoints = self.generate_lawnmower_waypoints()
        print(f"📋 Generated {len(waypoints)} waypoints")
        
        self.metrics.start()
        
        for i, (lat, lon) in enumerate(waypoints):
            if len(self.discovered_tags) >= len(self.ble_simulator.tags):
                print("✅ All tags found!")
                break
            
            if time.time() - self.metrics.start_time > MAX_MISSION_TIME:
                print("⏱️ Time limit!")
                break
            
            print(f"➡️ Waypoint {i+1}/{len(waypoints)}")
            await self.goto(lat, lon)
            await self.wait_arrival(lat, lon)
        
        self.metrics.stop()
        await self.return_home()
        
        return self.metrics.get_summary()


class GPMission(BaseMission):
    """GP + UCB + Coverage exploration."""
    
    async def run(self):
        print("\n" + "=" * 50)
        print("🧠 GP INTELLIGENT MISSION")
        print("=" * 50)
        
        await self.connect()
        await self.setup()
        await self.arm_and_takeoff()
        
        planner = AdaptivePlanner(
            origin_lat=self.home_lat,
            origin_lon=self.home_lon,
            area_size=150.0
        )
        
        self.metrics.start()
        
        while True:
            await self.update_position()
            self.metrics.add_position(self.current_lat, self.current_lon)
            readings = self.scan()
            
            if readings:
                rssi = max(r['rssi'] for r in readings)
            else:
                rssi = -95
            
            planner.add_observation(self.current_lat, self.current_lon, rssi)
            
            if len(self.discovered_tags) >= len(self.ble_simulator.tags):
                print("✅ All tags found!")
                break
            
            if time.time() - self.metrics.start_time > MAX_MISSION_TIME:
                print("⏱️ Time limit!")
                break
            
            target, score = planner.get_next_waypoint()
            if target:
                print(f"📍 Target: UCB={score:.1f}")
                await self.goto(target[0], target[1])
                await self.wait_arrival(target[0], target[1], timeout=45)
        
        self.metrics.stop()
        await self.return_home()
        
        return self.metrics.get_summary()


def print_comparison(lawnmower_results, gp_results):
    """Print side-by-side comparison."""
    print("\n" + "=" * 60)
    print("📊 BENCHMARK RESULTS")
    print("=" * 60)
    
    print(f"\n{'Metric':<25} {'Lawnmower':<15} {'GP + UCB':<15} {'Winner':<10}")
    print("-" * 60)
    
    # Duration
    lm_dur = lawnmower_results['duration_s']
    gp_dur = gp_results['duration_s']
    winner = "GP ✓" if gp_dur < lm_dur else "Lawnmower ✓" if lm_dur < gp_dur else "Tie"
    print(f"{'Duration (s)':<25} {lm_dur:<15} {gp_dur:<15} {winner:<10}")
    
    # Tags found
    lm_tags = lawnmower_results['tags_found']
    gp_tags = gp_results['tags_found']
    winner = "GP ✓" if gp_tags > lm_tags else "Lawnmower ✓" if lm_tags > gp_tags else "Tie"
    print(f"{'Tags Found':<25} {lm_tags:<15} {gp_tags:<15} {winner:<10}")
    
    # Distance
    lm_dist = lawnmower_results['distance_m']
    gp_dist = gp_results['distance_m']
    winner = "GP ✓" if gp_dist < lm_dist else "Lawnmower ✓" if lm_dist < gp_dist else "Tie"
    print(f"{'Distance (m)':<25} {lm_dist:<15} {gp_dist:<15} {winner:<10}")
    
    # Efficiency
    lm_eff = lawnmower_results['efficiency']
    gp_eff = gp_results['efficiency']
    winner = "GP ✓" if gp_eff > lm_eff else "Lawnmower ✓" if lm_eff > gp_eff else "Tie"
    print(f"{'Efficiency (tags/100m)':<25} {lm_eff:<15} {gp_eff:<15} {winner:<10}")
    
    print("=" * 60)


async def main():
    parser = argparse.ArgumentParser(description='Benchmark missions')
    parser.add_argument('--mode', choices=['lawnmower', 'gp', 'both'], 
                       default='both', help='Mission mode')
    args = parser.parse_args()
    
    lawnmower_results = None
    gp_results = None
    
    if args.mode in ['lawnmower', 'both']:
        mission = LawnmowerMission()
        lawnmower_results = await mission.run()
        lawnmower_results['mode'] = 'Lawnmower'
        print(f"\n📊 Lawnmower: {lawnmower_results}")
    
    if args.mode == 'both':
        print("\n⏳ Waiting 10s before next mission...")
        await asyncio.sleep(10)
    
    if args.mode in ['gp', 'both']:
        mission = GPMission()
        gp_results = await mission.run()
        gp_results['mode'] = 'GP'
        print(f"\n📊 GP: {gp_results}")
    
    if args.mode == 'both' and lawnmower_results and gp_results:
        print_comparison(lawnmower_results, gp_results)


if __name__ == "__main__":
    asyncio.run(main())
