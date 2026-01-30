# particle_visualizer.py
"""
Live Particle Filter Visualization Manager
==========================================
Pre-allocates a fixed grid of subplots. As tags are discovered,
they fill in the empty slots dynamically.
"""

import matplotlib.pyplot as plt
import numpy as np
import math


class ParticleVisualizationManager:
    """
    Manages a live visualization window with pre-allocated subplots.
    Tags fill in empty slots as they are discovered.
    """
    
    def __init__(self, home_lat=47.397971, home_lon=8.546164, bounds=None, 
                 max_tags=10, n_cols=5):
        """
        Initialize the visualization manager with pre-allocated subplots.
        
        Args:
            home_lat: Reference latitude for local coordinate conversion
            home_lon: Reference longitude for local coordinate conversion
            bounds: Dict with lat_min, lat_max, long_min, long_max
            max_tags: Maximum number of tags to display (default 10)
            n_cols: Number of columns in the grid (default 5)
        """
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.max_tags = max_tags
        self.n_cols = n_cols
        self.n_rows = math.ceil(max_tags / n_cols)
        self.R = 6371000
        
        # Track registered localizers: {tag_id: (localizer, subplot_index)}
        self.localizers = {}
        self.next_slot = 0
        
        # Fixed axis limits
        if bounds:
            x_min, y_min = self._gps_to_local(bounds['lat_min'], bounds['long_min'])
            x_max, y_max = self._gps_to_local(bounds['lat_max'], bounds['long_max'])
            pad_x = (x_max - x_min) * 0.1
            pad_y = (y_max - y_min) * 0.1
            self.axis_limits = (x_min - pad_x, x_max + pad_x, y_min - pad_y, y_max + pad_y)
        else:
            self.axis_limits = (-120, 120, -120, 120)
        
        # Create the figure with all subplots upfront
        plt.ion()
        self.fig, self.axes_array = plt.subplots(
            self.n_rows, self.n_cols,
            figsize=(4 * self.n_cols, 3 * self.n_rows),
            num='ParticleFilter'
        )
        self.fig.suptitle('Particle Filter Localization', fontsize=16, fontweight='bold')
        
        # Initialize all axes as empty placeholders
        for row in range(self.n_rows):
            for col in range(self.n_cols):
                ax = self.axes_array[row, col]
                ax.set_xlim(self.axis_limits[0], self.axis_limits[1])
                ax.set_ylim(self.axis_limits[2], self.axis_limits[3])
                ax.set_title('[ Empty ]', fontsize=11, color='gray')
                ax.set_xlabel('East (m)', fontsize=8)
                ax.set_ylabel('North (m)', fontsize=8)
                ax.grid(True, alpha=0.2)
                ax.tick_params(labelsize=7)
        
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        self.fig.canvas.draw()
        plt.pause(0.1)
    
    def _gps_to_local(self, lat, lon):
        """Convert GPS coordinates to local meters relative to home."""
        x = self.R * math.radians(lon - self.home_lon) * math.cos(math.radians(self.home_lat))
        y = self.R * math.radians(lat - self.home_lat)
        return x, y
    
    def register_tag(self, tag_id, localizer):
        """
        Register a new tag - assigns it to the next available subplot.
        """
        if tag_id in self.localizers:
            return  # Already registered
        
        if self.next_slot >= self.max_tags:
            print(f"Warning: Max tags ({self.max_tags}) reached, cannot add {tag_id}")
            return
        
        # Assign to next available slot
        row = self.next_slot // self.n_cols
        col = self.next_slot % self.n_cols
        ax = self.axes_array[row, col]
        
        # Store localizer with its assigned axis
        self.localizers[tag_id] = {'localizer': localizer, 'ax': ax}
        
        # Update the axis title to show the tag name
        ax.set_title(f'[{tag_id}]', fontsize=12, fontweight='bold', color='darkblue')
        
        self.next_slot += 1
        self.fig.canvas.draw_idle()
    
    def update(self, tag_id=None, drone_pos=None):
        """Update the visualization for one or all tags."""
        if len(self.localizers) == 0:
            return
        
        tags_to_update = [tag_id] if tag_id else list(self.localizers.keys())
        
        for tid in tags_to_update:
            if tid not in self.localizers:
                continue
            
            data = self.localizers[tid]
            ax = data['ax']
            localizer = data['localizer']
            
            # Clear and redraw
            ax.clear()
            ax.set_title(f'[{tid}]', fontsize=12, fontweight='bold', color='darkblue')
            ax.set_xlabel('East (m)', fontsize=8)
            ax.set_ylabel('North (m)', fontsize=8)
            ax.set_xlim(self.axis_limits[0], self.axis_limits[1])
            ax.set_ylim(self.axis_limits[2], self.axis_limits[3])
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=7)
            
            # Get particle data
            particles = localizer.particles
            particle_lats = particles[:, 0]
            particle_lons = particles[:, 1]
            weights = particles[:, 2]
            
            # Convert to local coordinates
            p_x = np.array([self._gps_to_local(lat, lon)[0] 
                           for lat, lon in zip(particle_lats, particle_lons)])
            p_y = np.array([self._gps_to_local(lat, lon)[1] 
                           for lat, lon in zip(particle_lats, particle_lons)])
            
            # Normalize weights for coloring
            w_norm = weights / np.max(weights) if np.max(weights) > 0 else weights
            
            # Plot particles - LARGER and more visible
            ax.scatter(p_x, p_y, c='blue', s=15, alpha=0.5, marker='.')
            
            # Plot estimated position (green X)
            est_lat, est_lon = localizer.estimate()
            est_x, est_y = self._gps_to_local(est_lat, est_lon)
            ax.scatter(est_x, est_y, c='lime', s=100, marker='x', 
                      linewidth=3, zorder=10, edgecolors='black')
            
            # Plot drone position (red triangle)
            if drone_pos:
                d_x, d_y = self._gps_to_local(drone_pos[0], drone_pos[1])
                ax.scatter(d_x, d_y, c='red', s=60, marker='^', zorder=10)
        
        # Refresh
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)
    
    def close(self):
        """Close the visualization window."""
        if self.fig is not None:
            plt.close(self.fig)
            self.fig = None


# Test standalone
if __name__ == "__main__":
    import sys
    import os
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    from localization.particle_localizer import ParticleLocalizer
    import time
    
    bounds = {
        'lat_min': 47.3970, 'lat_max': 47.3990,
        'long_min': 8.5450, 'long_max': 8.5470
    }
    
    viz = ParticleVisualizationManager(bounds=bounds, max_tags=10, n_cols=5)
    
    tag_ids = ['TAG_001', 'TAG_002', 'TAG_003', 'TAG_004', 'TAG_005']
    localizers = {}
    
    for tag_id in tag_ids:
        print(f"Found {tag_id}!")
        localizers[tag_id] = ParticleLocalizer(num_particles=200, area_bounds=bounds)
        viz.register_tag(tag_id, localizers[tag_id])
        
        for j in range(5):
            angle = j * 0.8
            drone_lat = 47.3980 + 0.0003 * np.sin(angle)
            drone_lon = 8.5460 + 0.0003 * np.cos(angle)
            rssi = -70 + np.random.normal(0, 3)
            
            localizers[tag_id].update(drone_lat, drone_lon, rssi)
            viz.update(tag_id, drone_pos=(drone_lat, drone_lon))
            time.sleep(0.05)
        
        time.sleep(0.5)
    
    print("Done! Close window to exit.")
    plt.ioff()
    plt.show()
