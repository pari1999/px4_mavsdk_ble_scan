"""
BLE Tag Simulator for BLE Scanner
Simulates virtual BLE tags and returns RSSI values based on drone position.
"""

import math
import random
import yaml


class BLETag:
    """Represents a single BLE tag with position and transmit power."""
    
    def __init__(self, tag_id, latitude, longitude, altitude=1.0, tx_power=-45.0):
        self.tag_id = tag_id
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.tx_power = tx_power
    
    def to_dict(self):
        """Convert to dictionary for serialization."""
        return {
            "tag_id": self.tag_id,
            "latitude": self.latitude,
            "longitude": self.longitude,
            "altitude": self.altitude,
            "tx_power": self.tx_power
        }
    
    def __str__(self):
        return f"BLETag({self.tag_id} at {self.latitude:.6f}, {self.longitude:.6f})"


class RSSIPropagationModel:
    """
    Models BLE signal strength using log-distance path loss.
    RSSI = TX_POWER - 10 * n * log10(distance) + noise
    """
    
    def __init__(self, path_loss_exponent=2.5, noise_std=4.0, 
                 min_rssi=-100.0, detection_threshold=-85.0):
        self.path_loss_exponent = path_loss_exponent
        self.noise_std = noise_std
        self.min_rssi = min_rssi
        self.detection_threshold = detection_threshold
    
    def calculate_rssi(self, distance, tx_power=-45.0):
        """Calculate RSSI at given distance with noise."""
        if distance < 0.1:
            distance = 0.1
        
        path_loss = 10 * self.path_loss_exponent * math.log10(distance)
        noise = random.gauss(0, self.noise_std)
        rssi = tx_power - path_loss + noise
        
        return max(rssi, self.min_rssi)
    
    def is_detected(self, rssi):
        """Check if RSSI is above detection threshold."""
        return rssi >= self.detection_threshold


class BLESimulator:
    """Manages BLE tags and provides RSSI readings based on drone position."""
    
    def __init__(self, propagation_model=None):
        if propagation_model is not None:
            self.model = propagation_model
        else:
            self.model = RSSIPropagationModel()
        
        self.tags = []
        self.total_queries = 0
        self.discovery_counts = {}
    
    def add_tag(self, tag):
        """Add a tag to the simulator."""
        self.tags.append(tag)
        self.discovery_counts[tag.tag_id] = 0
    
    def add_tags(self, tags):
        """Add multiple tags."""
        for tag in tags:
            self.add_tag(tag)
    
    def generate_random_tags(self, num_tags, center_lat, center_lon, 
                             radius_meters=25.0, altitude=1.5):
        """Generate random tags around a center point."""
        lat_per_meter = 1.0 / 111111.0
        lon_per_meter = 1.0 / (111111.0 * math.cos(math.radians(center_lat)))
        
        for i in range(num_tags):
            angle = random.uniform(0, 2 * math.pi)
            dist = random.uniform(0, radius_meters)
            
            delta_lat = dist * math.cos(angle) * lat_per_meter
            delta_lon = dist * math.sin(angle) * lon_per_meter
            
            tag = BLETag(
                tag_id=f"TAG_{i + 1:03d}",
                latitude=center_lat + delta_lat,
                longitude=center_lon + delta_lon,
                altitude=altitude
            )
            self.add_tag(tag)
    
    def _calculate_distance(self, drone_lat, drone_lon, drone_alt, tag):
        """Calculate 3D distance between drone and tag using Haversine."""
        R = 6371000  # Earth radius in meters
        
        phi1 = math.radians(drone_lat)
        phi2 = math.radians(tag.latitude)
        delta_phi = math.radians(tag.latitude - drone_lat)
        delta_lambda = math.radians(tag.longitude - drone_lon)
        
        a = (math.sin(delta_phi / 2) ** 2 + 
             math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        horizontal_dist = R * c
        vertical_dist = abs(drone_alt - tag.altitude)
        
        return math.sqrt(horizontal_dist ** 2 + vertical_dist ** 2)
    
    def get_readings(self, drone_lat, drone_lon, drone_alt):
        """Get RSSI readings for all tags from drone's current position."""
        self.total_queries += 1
        readings = []
        
        for tag in self.tags:
            distance = self._calculate_distance(drone_lat, drone_lon, drone_alt, tag)
            rssi = self.model.calculate_rssi(distance, tag.tx_power)
            detected = self.model.is_detected(rssi)
            
            if detected:
                self.discovery_counts[tag.tag_id] += 1
            
            readings.append({
                "tag_id": tag.tag_id,
                "rssi": round(rssi, 1),
                "distance": round(distance, 2),
                "detected": detected,
                "tag_lat": tag.latitude,
                "tag_lon": tag.longitude
            })
        
        return readings
    
    def get_detected_tags(self, drone_lat, drone_lon, drone_alt):
        """Get only tags above detection threshold."""
        all_readings = self.get_readings(drone_lat, drone_lon, drone_alt)
        return [r for r in all_readings if r["detected"]]
    
    def get_strongest_signal(self, drone_lat, drone_lon, drone_alt):
        """Get tag with strongest RSSI, or None."""
        detected = self.get_detected_tags(drone_lat, drone_lon, drone_alt)
        if not detected:
            return None
        return max(detected, key=lambda r: r["rssi"])
    
    def get_statistics(self):
        """Get simulation statistics."""
        tags_detected = sum(1 for c in self.discovery_counts.values() if c > 0)
        return {
            "total_tags": len(self.tags),
            "total_queries": self.total_queries,
            "tags_detected": tags_detected
        }
    
    def save_to_yaml(self, file_path):
        """Save configuration to YAML file."""
        config = {
            "tags": [tag.to_dict() for tag in self.tags],
            "propagation_model": {
                "path_loss_exponent": self.model.path_loss_exponent,
                "noise_std": self.model.noise_std,
                "detection_threshold": self.model.detection_threshold
            }
        }
        with open(file_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
    
    def load_from_yaml(self, file_path):
        """Load configuration from YAML file."""
        with open(file_path, 'r') as f:
            config = yaml.safe_load(f)
        
        model_cfg = config.get("propagation_model", {})
        self.model = RSSIPropagationModel(
            path_loss_exponent=model_cfg.get("path_loss_exponent", 2.5),
            noise_std=model_cfg.get("noise_std", 4.0),
            detection_threshold=model_cfg.get("detection_threshold", -85.0)
        )
        
        self.tags = []
        for tag_cfg in config.get("tags", []):
            tag = BLETag(
                tag_id=tag_cfg["tag_id"],
                latitude=tag_cfg["latitude"],
                longitude=tag_cfg["longitude"],
                altitude=tag_cfg.get("altitude", 1.0),
                tx_power=tag_cfg.get("tx_power", -45.0)
            )
            self.add_tag(tag)


if __name__ == "__main__":
    SITL_LAT = 47.397742
    SITL_LON = 8.545594
    
    print("=" * 50)
    print("BLE Tag Simulator Test")
    print("=" * 50)
    
    simulator = BLESimulator()
    simulator.generate_random_tags(10, SITL_LAT, SITL_LON, radius_meters=30.0)
    simulator.save_to_yaml("tags_config.yaml")
    
    print(f"\nGenerated {len(simulator.tags)} tags")
    
    print("\nDrone at origin, 10m altitude:")
    readings = simulator.get_readings(SITL_LAT, SITL_LON, 10.0)
    
    detected = [r for r in readings if r["detected"]]
    print(f"Detected: {len(detected)}/{len(readings)} tags")
    
    for r in readings:
        status = "✓" if r["detected"] else "✗"
        print(f"  {status} {r['tag_id']}: {r['rssi']:6.1f} dBm ({r['distance']:.1f}m)")
    
    strongest = simulator.get_strongest_signal(SITL_LAT, SITL_LON, 10.0)
    if strongest:
        print(f"\nStrongest: {strongest['tag_id']} @ {strongest['rssi']:.1f} dBm")
