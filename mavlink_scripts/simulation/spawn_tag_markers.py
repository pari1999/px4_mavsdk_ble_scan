"""
Spawn visual tag markers in Gazebo at tag locations.
Creates red cylinders at GPS coordinates from tags_config.yaml.

Usage:
    python3 spawn_tag_markers.py
"""

import subprocess
import math
import yaml
import time
import os


# Gazebo world origin (from default.sdf)
WORLD_ORIGIN_LAT = 47.397971057728974
WORLD_ORIGIN_LON = 8.546163739800146

# Path to the marker model file
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Models are in the parent directory (mavlink_scripts/models)
MARKER_MODEL_PATH = os.path.join(os.path.dirname(SCRIPT_DIR), "models", "tag_marker", "model.sdf")


def gps_to_local(lat, lon, origin_lat=WORLD_ORIGIN_LAT, origin_lon=WORLD_ORIGIN_LON):
    """Convert GPS to local ENU coordinates (meters from origin)."""
    R = 6371000
    
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    origin_lat_rad = math.radians(origin_lat)
    origin_lon_rad = math.radians(origin_lon)
    
    x = R * (lon_rad - origin_lon_rad) * math.cos(origin_lat_rad)
    y = R * (lat_rad - origin_lat_rad)
    
    return x, y


def spawn_marker(name, x, y, z):
    """Spawn a marker at (x, y, z) in Gazebo."""
    cmd = [
        "gz", "service", 
        "-s", "/world/default/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "3000",
        "--req", f'sdf_filename: "{MARKER_MODEL_PATH}" name: "{name}" pose: {{position: {{x: {x} y: {y} z: {z}}}}}'
    ]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        return "data: true" in result.stdout
    except Exception as e:
        print(f"    Error: {e}")
        return False


def load_tags(config_file="tags_config.yaml"):
    """Load tags from YAML file."""
    config_path = os.path.join(SCRIPT_DIR, config_file)
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config.get("tags", [])


def main():
    print("=" * 50)
    print("Spawning Tag Markers in Gazebo")
    print("=" * 50)
    
    # Check model file exists
    if not os.path.exists(MARKER_MODEL_PATH):
        print(f"Error: Model file not found at {MARKER_MODEL_PATH}")
        return
    
    # Load tags
    try:
        tags = load_tags()
        print(f"\nLoaded {len(tags)} tags")
    except FileNotFoundError:
        print("Error: tags_config.yaml not found!")
        return
    
    print(f"World origin: ({WORLD_ORIGIN_LAT:.6f}, {WORLD_ORIGIN_LON:.6f})")
    print("\nSpawning markers...")
    
    success = 0
    for tag in tags:
        tag_id = tag["tag_id"]
        lat = tag["latitude"]
        lon = tag["longitude"]
        alt = tag.get("altitude", 1.0)
        
        x, y = gps_to_local(lat, lon)
        z = alt
        
        name = f"marker_{tag_id}"
        print(f"  {tag_id}: ({x:.1f}, {y:.1f}, {z:.1f})", end=" ")
        
        if spawn_marker(name, x, y, z):
            print("✓")
            success += 1
        else:
            print("✗")
        
        time.sleep(0.3)
    
    print(f"\n{'=' * 50}")
    print(f"Spawned {success}/{len(tags)} markers")
    print("=" * 50)


if __name__ == "__main__":
    main()
