"""
Intelligent Mission Controller for BLE Scanner
=================================================
Simplest version using goto_location() with direct GPS coordinates.

Features:
- GP + UCB exploration strategy
- Direct GPS waypoint navigation (no offboard mode)
- Real-hardware-safe parameters
"""

import asyncio
import time
import math
import json
import websockets
import sys
import os
# Add parent directory to path to allow importing from sibling packages
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from mavsdk import System

from simulation.ble_simulator import BLESimulator
from localization.gaussian_process import AdaptivePlanner
from localization.particle_localizer import ParticleLocalizer
from interface.particle_visualizer import ParticleVisualizationManager

# Configuration
SITL_CONNECTION = "udp://:14540"
# For real hardware via USB: "serial:///dev/ttyACM0:921600"
# For real hardware via UART: "serial:///dev/ttyAMA0:57600"

# Flight parameters (safe values for autonomous operation)
FLIGHT_ALTITUDE = 15.0          # meters - safe height for scanning
MAX_SPEED = 2.5                 # m/s - conservative for reliability
RETURN_SPEED = 5.0              # m/s - faster for RTL (open path)
WAYPOINT_ARRIVAL_THRESHOLD = 5.0  # meters

# Mission timing
MAX_MISSION_TIME = 300          # seconds (5 minutes)
SCAN_RATE = 2.0                 # Hz
NEW_WAYPOINT_COOLDOWN = 20.0    # seconds min between changes
WAYPOINT_TIMEOUT = 45.0         # seconds max at one waypoint


class MissionState:
    """Possible mission states."""
    INIT = "INIT"
    TAKEOFF = "TAKEOFF"
    EXPLORE = "EXPLORE"
    RETURNING = "RETURNING"
    LANDING = "LANDING"
    DONE = "DONE"


class IntelligentMission:
    """Main mission controller using direct GPS navigation."""
    
    def __init__(self, connection_url=SITL_CONNECTION):
        self.connection_url = connection_url
        
        # Drone
        self.drone = None
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        
        # Dashboard Connection
        self.websocket = None
        self.dashboard_url = "ws://localhost:8765"
        self.absolute_altitude = 0.0  # AMSL altitude
        
        # BLE Simulator
        self.ble_simulator = BLESimulator()
        
        # Planner
        self.planner = None
        
        # Mission state
        self.state = MissionState.INIT
        self.discovered_tags = set()
        self.scan_count = 0
        
        # Waypoint management
        self.current_target = None
        self.last_waypoint_time = 0
        
        # Path logging for visualization
        self.path_log = []  # [(lat, lon, time, rssi)]
        self.waypoint_log = []  # [(lat, lon, ucb_score, time)]
        
        # Timing
        self.mission_start_time = 0
    
    async def connect_dashboard(self):
        """Connect to the WebSocket dashboard server."""
        try:
            self.websocket = await websockets.connect(self.dashboard_url)
            print(f"✅ Connected to dashboard at {self.dashboard_url}")
        except Exception as e:
            print(f"⚠️ Dashboard connection failed: {e}")
            self.websocket = None

    async def broadcast_state(self, latest_readings=None):
        """Send mission state to dashboard."""
        if not self.websocket:
            return

        # Prepare list of active tags
        tags_data = []
        
        # 1. Add detected tags with signal strength
        if latest_readings:
            for r in latest_readings:
                tag_id = r['tag_id']
                
                # Get estimated position if available
                est_lat, est_lon = 0.0, 0.0
                if tag_id in self.localisers:
                    est_lat, est_lon = self.localisers[tag_id].estimate()
                
                tags_data.append({
                    "id": tag_id,
                    "rssi": float(r['rssi']), # Convert numpy float if needed
                    "est_lat": float(est_lat),
                    "est_lon": float(est_lon)
                })

        state = {
            "mode": self.state,
            "drone": {
                "lat": float(self.current_lat),
                "lon": float(self.current_lon),
                "alt": float(self.current_alt)
            },
            "tags": tags_data,
            "stats": {
                "found": len(self.discovered_tags),
                "total": len(self.ble_simulator.tags)
            },
            "sim_time": round(time.time() - self.mission_start_time, 1)
        }
        
        try:
            await self.websocket.send(json.dumps(state))
        except Exception as e:
            print(f"⚠️ Dashboard send failed: {e}")

    async def connect(self):
        """Connect to the drone."""
        # Connect dashboard first
        await self.connect_dashboard()

        print(f"📡 Connecting to {self.connection_url}...")
        self.drone = System()
        await self.drone.connect(system_address=self.connection_url)
        
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("✅ Connected!")
                return True
        return False
    
    async def update_position(self):
        """Get current drone position."""
        async for pos in self.drone.telemetry.position():
            self.current_lat = pos.latitude_deg
            self.current_lon = pos.longitude_deg
            self.current_alt = pos.relative_altitude_m
            self.absolute_altitude = pos.absolute_altitude_m
            return
    
    async def setup(self):
        """Initialize components after connecting."""
        await self.update_position()
        print(f"📍 Home: ({self.current_lat:.6f}, {self.current_lon:.6f})")
        print(f"📍 Ground AMSL: {self.absolute_altitude:.1f}m")
        
        self.planner = AdaptivePlanner(
            origin_lat=self.current_lat,
            origin_lon=self.current_lon,
            area_size=150.0
        )
        
        try:
            self.ble_simulator.load_from_yaml("tags_config.yaml")
            print(f"✅ Loaded {len(self.ble_simulator.tags)} tags")
        except FileNotFoundError:
            print("⚠️ Generating random tags...")
            self.ble_simulator.generate_random_tags(
                10, self.current_lat, self.current_lon, 150.0
            )
            self.ble_simulator.save_to_yaml("tags_config.yaml")
            
    
    async def arm_and_takeoff(self):
        """Arm and take off with safe speed settings."""
        self.state = MissionState.TAKEOFF
        print("\n🚁 Pre-flight setup...")
        
        # Wait for GPS
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok:
                break
        
        # Set safe speed limit
        print(f"   Setting speed: {MAX_SPEED} m/s")
        await self.drone.action.set_current_speed(MAX_SPEED)
        
        # Set return altitude (for RTL)
        await self.drone.action.set_return_to_launch_altitude(FLIGHT_ALTITUDE + 5)
        
        print("🚁 Taking off...")
        await self.drone.action.arm()
        await self.drone.action.set_takeoff_altitude(FLIGHT_ALTITUDE)
        await self.drone.action.takeoff()
        
        target_alt = FLIGHT_ALTITUDE * 0.9
        while True:
            await self.update_position()
            if self.current_alt >= target_alt:
                break
            await asyncio.sleep(0.5)
        
        print(f"✅ At altitude: {self.current_alt:.1f}m (AMSL: {self.absolute_altitude:.1f}m)")
    
    def scan(self):
        """Perform one RSSI scan and return readings."""
        readings = self.ble_simulator.get_detected_tags(
            self.current_lat, self.current_lon, self.current_alt
        )
        self.scan_count += 1
        
        new_discoveries = []
        for r in readings:
            tag_id = r["tag_id"]
            if tag_id not in self.discovered_tags:
                self.discovered_tags.add(tag_id)
                new_discoveries.append(tag_id)
                print(f"🏷️  FOUND: {tag_id} @ {r['rssi']:.1f} dBm")
        
        return readings, new_discoveries
    
    def get_distance_to_target(self):
        """Calculate distance to current target in meters."""
        if self.current_target is None:
            return float('inf')
        
        target_lat, target_lon = self.current_target
        
        R = 6371000
        dlat = math.radians(target_lat - self.current_lat)
        dlon = math.radians(target_lon - self.current_lon)
        
        a = math.sin(dlat/2)**2 + math.cos(math.radians(self.current_lat)) * \
            math.cos(math.radians(target_lat)) * math.sin(dlon/2)**2
        
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    def should_get_new_waypoint(self):
        """Decide if we should calculate a new waypoint."""
        if self.current_target is None:
            return True
        
        if self.get_distance_to_target() < WAYPOINT_ARRIVAL_THRESHOLD:
            return True
        
        if time.time() - self.last_waypoint_time > WAYPOINT_TIMEOUT:
            return True
        
        if time.time() - self.last_waypoint_time < NEW_WAYPOINT_COOLDOWN:
            return False
        
        return False
    
    async def fly_to_target(self):
        """Command drone to fly to target GPS coordinates."""
        if self.current_target is None:
            return
        
        target_lat, target_lon = self.current_target
        
        # Calculate target AMSL altitude (ground + desired height)
        # Use current absolute altitude as reference
        target_amsl = self.absolute_altitude - self.current_alt + FLIGHT_ALTITUDE
        
        await self.drone.action.goto_location(
            target_lat,
            target_lon,
            target_amsl,
            0  # Yaw: 0 = North (or use NaN for no yaw control)
        )
    
    async def explore_loop(self):
        """Main exploration loop."""
        self.state = MissionState.EXPLORE
        self.mission_start_time = time.time()
        self.localisers = {}
        
        # Initialize live particle visualization with fixed bounds
        sitl_bounds = {
            'lat_min': 47.3970, 'lat_max': 47.3990,
            'long_min': 8.5450, 'long_max': 8.5470
        }
        self.particle_viz = ParticleVisualizationManager(
            home_lat=self.current_lat, home_lon=self.current_lon,
            bounds=sitl_bounds
        )
        
        print("\n🔍 Starting exploration (GPS mode)...")
        
        loop_interval = 1.0 / SCAN_RATE
        
        while self.state == MissionState.EXPLORE:
            loop_start = time.time()
            
            await self.update_position()
            readings, _ = self.scan()

            #format can be a dictionary where in the key is the tag id and the  value is its localiser object
            for r in readings:
                if r['tag_id'] not in self.localisers.keys():
                    # Define bounds for the localizer (Zurich SITL area)
                    sitl_bounds = {
                        'lat_min': 47.3970, 'lat_max': 47.3990,
                        'long_min': 8.5450, 'long_max': 8.5470
                    }
                    self.localisers[r['tag_id']] = ParticleLocalizer(area_bounds=sitl_bounds)
                    # Register with live visualizer
                    self.particle_viz.register_tag(r['tag_id'], self.localisers[r['tag_id']])
                
                self.localisers[r['tag_id']].update(self.current_lat, self.current_lon, r['rssi'])
                # Update live visualization only for the tag that got a reading
                self.particle_viz.update(r['tag_id'], drone_pos=(self.current_lat, self.current_lon))

            # Update visualization for ALL registered tags (smooth display)
            # Note: We reverted this in previous step but now we want dashboard update
            if self.websocket:
                await self.broadcast_state(latest_readings=readings)

            if readings:
                strongest = max(readings, key=lambda r: r['rssi'])
                rssi = strongest['rssi']
            else:
                rssi = -95
            
            # Log path
            elapsed = time.time() - self.mission_start_time
            self.path_log.append((self.current_lat, self.current_lon, elapsed, rssi))
            
            self.planner.add_observation(self.current_lat, self.current_lon, rssi)
            
            if self.should_get_new_waypoint():
                new_target, score = self.planner.get_next_waypoint()
                if new_target:
                    self.current_target = new_target
                    self.last_waypoint_time = time.time()
                    dist = self.get_distance_to_target()
                    # Log waypoint
                    self.waypoint_log.append((new_target[0], new_target[1], score, elapsed))
                    print(f"📍 Target: ({new_target[0]:.6f}, {new_target[1]:.6f}) | {dist:.1f}m | UCB: {score:.1f}")
                    await self.fly_to_target()
            
            elapsed = time.time() - self.mission_start_time
            total_tags = len(self.ble_simulator.tags)
            
            if len(self.discovered_tags) >= total_tags:
                print(f"\n✅ All {total_tags} tags found!")
                self.state = MissionState.RETURNING
                break
            
            if elapsed > MAX_MISSION_TIME:
                print(f"\n⏱️ Time limit reached ({MAX_MISSION_TIME}s)")
                self.state = MissionState.RETURNING
                break
            
            if self.scan_count % 20 == 0:
                dist = self.get_distance_to_target()
                print(f"📊 Found: {len(self.discovered_tags)}/{total_tags} | Dist: {dist:.1f}m | Time: {elapsed:.0f}s")
            
            elapsed_loop = time.time() - loop_start
            if elapsed_loop < loop_interval:
                await asyncio.sleep(loop_interval - elapsed_loop)
    
    async def return_and_land(self):
        """Return to home and land."""
        self.state = MissionState.LANDING
        print("\n🏠 Returning to launch...")
        
        await self.drone.action.return_to_launch()
        
        async for in_air in self.drone.telemetry.in_air():
            if not in_air:
                break
        
        print("✅ Landed!")
        self.state = MissionState.DONE
    
    def print_results(self):
        """Print mission summary."""
        elapsed = time.time() - self.mission_start_time
        total_tags = len(self.ble_simulator.tags)
        found = len(self.discovered_tags)
        
        print("\n" + "=" * 50)
        print("📊 MISSION RESULTS")
        print("=" * 50)
        print(f"   Duration: {elapsed:.1f} seconds")
        print(f"   Scans: {self.scan_count}")
        print(f"   Tags found: {found}/{total_tags} ({found/total_tags*100:.0f}%)")
        print(f"   Discovered: {sorted(self.discovered_tags)}")
        print("=" * 50)
    
    def visualize_path(self, filename="mission_path.png"):
        """Generate visualization of the mission path."""
        import matplotlib.pyplot as plt
        import numpy as np
        
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        
        # Get data
        home_lat, home_lon = self.path_log[0][0], self.path_log[0][1]
        
        # Convert to local meters
        R = 6371000
        path_x = [R * math.radians(p[1] - home_lon) * math.cos(math.radians(home_lat)) for p in self.path_log]
        path_y = [R * math.radians(p[0] - home_lat) for p in self.path_log]
        path_rssi = [p[3] for p in self.path_log]
        
        wp_x = [R * math.radians(w[1] - home_lon) * math.cos(math.radians(home_lat)) for w in self.waypoint_log]
        wp_y = [R * math.radians(w[0] - home_lat) for w in self.waypoint_log]
        wp_ucb = [w[2] for w in self.waypoint_log]
        
        tag_x = [R * math.radians(t.longitude - home_lon) * math.cos(math.radians(home_lat)) for t in self.ble_simulator.tags]
        tag_y = [R * math.radians(t.latitude - home_lat) for t in self.ble_simulator.tags]
        
        # Plot 1: Flight path with RSSI coloring
        ax1 = axes[0]
        scatter = ax1.scatter(path_x, path_y, c=path_rssi, cmap='RdYlGn', s=10, alpha=0.7)
        ax1.plot(path_x, path_y, 'b-', alpha=0.3, linewidth=1, label='Flight path')
        ax1.scatter(tag_x, tag_y, c='red', s=100, marker='*', label='Actual Tags', zorder=10)
        ax1.scatter(0, 0, c='green', s=150, marker='H', label='Home', zorder=10)

        # Plot Estimated Tag Locations
        est_lat = []
        est_lon = []
        for tid, loc in self.localisers.items():
            l, ln = loc.estimate()
            est_lat.append(l)
            est_lon.append(ln)
        
        if est_lat:
            est_x = [R * math.radians(ln - home_lon) * math.cos(math.radians(home_lat)) for ln in est_lon]
            est_y = [R * math.radians(l - home_lat) for l in est_lat]
            ax1.scatter(est_x, est_y, c='blue', s=100, marker='x', linewidth=2, label='Estimated Tags', zorder=11)
            
            # Draw lines between actual and estimated for error visualization
            # (Simple matching by closest neighbor or known ID if possible)
            # For now just plot the points.
        
        # Draw waypoints with numbers
        for i, (x, y) in enumerate(zip(wp_x, wp_y)):
            ax1.annotate(str(i+1), (x, y), fontsize=8, ha='center', va='center',
                        bbox=dict(boxstyle='circle', facecolor='yellow', alpha=0.7))
        
        ax1.set_xlabel('East (m)')
        ax1.set_ylabel('North (m)')
        ax1.set_title('Flight Path (color = RSSI strength)')
        ax1.legend(loc='upper right')
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        plt.colorbar(scatter, ax=ax1, label='RSSI (dBm)')
        
        # Plot 2: UCB score over time for each waypoint
        ax2 = axes[1]
        times = [w[3] for w in self.waypoint_log]
        ax2.stem(times, wp_ucb, linefmt='b-', markerfmt='bo', basefmt='r-')
        for i, (t, u) in enumerate(zip(times, wp_ucb)):
            ax2.annotate(f'WP{i+1}', (t, u), textcoords='offset points', xytext=(5, 5), fontsize=8)
        
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('UCB Score')
        ax2.set_title('UCB Score at Each Waypoint Decision')
        ax2.grid(True, alpha=0.3)
        
        # Add explanation text
        fig.text(0.5, 0.02, 
                'High UCB = Explore (uncertain area) OR Exploit (known strong signal). '
                'Decreasing UCB over time shows GP learning the space.',
                ha='center', fontsize=9, style='italic')
        
        plt.tight_layout()
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"\n📈 Path visualization saved to: {filename}")
        plt.close()
    
    async def run(self):
        """Run the full mission."""
        print("\n" + "=" * 50)
        print("🚀 INTELLIGENT BLE SCANNER MISSION")
        print("=" * 50)
        
        try:
            await self.connect()
            await self.setup()
            await self.arm_and_takeoff()
            await self.explore_loop()
            await self.return_and_land()
            self.print_results()
            self.visualize_path()

            for tag_id, localizer in self.localisers.items():
                lat ,long = localizer.estimate()
                print(f"Tag {tag_id}: {lat}, {long}")
            
        except KeyboardInterrupt:
            print("\n⚠️ Mission interrupted!")
            await self.drone.action.return_to_launch()
        
        except Exception as e:
            print(f"\n❌ Error: {e}")
            raise


async def main():
    mission = IntelligentMission()
    await mission.run()


if __name__ == "__main__":
    asyncio.run(main())
