# particle_localizer.py
#This script is to create a particle filter that would estimate the postion of each tags
#and finally return the estimated positions of each tag

import numpy as np

import matplotlib.pyplot as plt



class ParticleLocalizer:

    def __init__(self, num_particles = 500,area_bounds = {'lat_min':-90,'lat_max':90,'long_min':-180,'long_max':180}):
        """
        create a new localiser for one tag

        Args:
            num_particles (int): number of particles to use in the particle filter
            area_bounds (dict): bounds of the area in which the tag is expected to be

        """
        self.num_particles = num_particles
        self.area_bounds = area_bounds
        self.particles = self._initialize_particles()

    def _initialize_particles(self):
        #initialise all the particles
        particles = np.random.uniform(low = [self.area_bounds['lat_min'],self.area_bounds['long_min']],
        high = [self.area_bounds['lat_max'],
        self.area_bounds['long_max']],size = (self.num_particles,2))

        weights = np.ones(self.num_particles) / self.num_particles
        particles = np.column_stack((particles,weights))

        return particles

    def get_rssi(self,drone_lat,drone_long):
        """
        get the RSSI value of the tag

        Returns:
            rssi (float): RSSI value of the tag
        """
        #take the distane of the derone and subtract it from the distance of the tag

    
    def update(self, drone_lat,drone_long,rssi):
        """
        update the particle filter with the new observation

        Args:
            drone_lat (float): latitude of the drone
            drone_long (float): longitude of the drone
            rssi (float): RSSI value of the tag
        """
        #first we will calculate the liklihood of each particle
        self._liklihood(drone_lat,drone_long,rssi)
        #then we will resample the particles based on the liklihood
        self._resample()
        # Note: visualization is now handled externally by ParticleVisualizationManager
        

    def _resample(self):
        """
        resample the particles based on the liklihood

        Args:
            particles (list): list of particles

        Returns:
            particles (list): resampled particles
        """
        #based on the calculated liklihood we will resample the particles
        #we will use the np.random.choice function to resample the particles
        #the probability of each particle being selected is proportional to its liklihood
        #normalised the weights
        weights = self.particles[:,2] / np.sum(self.particles[:,2])
        indices = np.random.choice(len(self.particles),size = self.num_particles,replace = True ,p=weights)
        self.particles = self.particles[indices]
        self.particles[:,2] = 1 / self.num_particles

    def _liklihood(self, drone_lat, drone_lon, rssi, noise=4.0):
        """
        calculate the liklihood of each particle

        Args:
            drone_lat (float): latitude of the drone
            drone_lon (float): longitude of the drone
            rssi (float): actual measured RSSI
            noise (float): standard deviation of measurement noise
        """
        distance = self._calculate_distance(drone_lat, drone_lon)
        
        # Calculate EXPECTED RSSI (Deterministic model, no random noise!)
        # RSSI = TxPower - 10 * n * log10(distance)
        # Using standard values: TxPower=-45, n=2.5
        # Use np.maximum to avoid log(0)
        expected = -45.0 - 10 * 2.5 * np.log10(np.maximum(distance, 0.1))
        
        error = np.abs(expected - rssi)
        
        # Calculate Gaussian likelihood
        liklihood = np.exp(-0.5 * (error / noise) ** 2)

        # Update weights
        self.particles[:, 2] = liklihood

    def _calculate_distance(self, drone_lat, drone_lon):
        """
        calculate the 3D distance between the drone and the particles
        Assumes:
        - Drone is flying at ~15m AGL (based on simulation_assumptions.md)
        - Tags are on the ground (0m AGL)
        
        Args:
            drone_lat (float): latitude of the drone
            drone_lon (float): longitude of the drone

        Returns:
            distance (float): 3D slant range between drone and particles
        """

        particle_lats = self.particles[:,0] #shape (500,1)
        particle_longs = self.particles[:,1] #shape (500,1)

        R = 6371000 #in m radius of the earth

        #convert the partile lats and longs to radians
        phi1 = np.radians(particle_lats)
        phi2 = np.radians(drone_lat)

        dphi = np.radians(np.subtract(particle_lats,drone_lat))
        dlambda = np.radians(np.subtract(particle_longs,drone_lon))

        a = np.sin(dphi/2)**2 + np.cos(phi1)*np.cos(phi2)*np.sin(dlambda/2)**2
        c = 2*np.arctan2(np.sqrt(a),np.sqrt(1-a))
        horizontal_dist = R * c
        
        # Calculate 3D slant range
        # Assumptions from simulation_assumptions.md
        drone_alt = 15.0  # Assumed flight altitude
        tag_alt = 0.0     # Assumed tag altitude
        vertical_dist = abs(drone_alt - tag_alt)
        
        distance = np.sqrt(horizontal_dist**2 + vertical_dist**2)

        return distance
        

    
    def estimate(self):
        """
        estimate the position of the tag

        Returns:
            lat (float): estimated latitude of the tag
            long (float): estimated longitude of the tag
        """
        lat = np.mean(self.particles[:,0])
        long = np.mean(self.particles[:,1])
        return lat,long

    def reset(self):
        """
        reset the particle filter
        """
        pass

    def visualization(self, actual_tag_pos=None, drone_pos=None):
        """
        Visualize the particles, estimated position, and actual tag
        """
        import matplotlib.pyplot as plt
        
        # Turn on interactive mode if not already on
        if not plt.isinteractive():
            plt.ion()
            
        plt.clf()
        
        # 1. Plot all particles (blue dots)
        plt.scatter(self.particles[:, 1], self.particles[:, 0], color='blue', s=5, alpha=0.5, label='Particles')
        
        # 2. Plot Estimated Position (green X)
        est_lat, est_lon = self.estimate()
        plt.scatter(est_lon, est_lat, color='green', marker='x', s=150, linewidth=3, label='Estimated')
        
        # 3. Plot Actual Tag Position (red star) - if known
        if actual_tag_pos:
            plt.scatter(actual_tag_pos[1], actual_tag_pos[0], color='red', marker='*', s=200, label='Actual Tag')
            
        # 4. Plot Drone Position (black pointer) - if known
        if drone_pos:
            plt.scatter(drone_pos[1], drone_pos[0], color='black', marker='^', s=100, label='Drone')
            
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.title(f'Particle Filter Localization (N={self.num_particles})')
        plt.legend(loc='upper right')
        plt.grid(True, alpha=0.3)
        plt.axis('equal') 
        
        plt.savefig("particle_filter_current.png")
        plt.draw()
        plt.pause(0.1) # Small pause to let the plot render

if __name__ == "__main__":
    
    # 1. Define area around Zurich (SITL Home)
    # Center: 47.397971, 8.546164
    bounds = {
        'lat_min': 47.3971, 'lat_max': 47.3988,
        'long_min': 8.5451, 'long_max': 8.5471
    }
    
    # Create localizer with localized bounds (so particles aren't worldwide)
    localizer = ParticleLocalizer(num_particles=500, area_bounds=bounds)
    
    # 2. Create a simulated tag (Actual position)
    actual_tag = (47.3975, 8.5458)
    
    # 3. Simulate Drone Movement (Circle around tag)
    print("Starting simulation...")
    
    # Generate points in a circle around the tag
    angles = np.linspace(0, 2*np.pi, 20)
    radius = 0.0003 # approx 30m in degrees
    
    try:
        for theta in angles:
            # Drone position
            drone_lat = actual_tag[0] + radius * np.sin(theta)
            drone_lon = actual_tag[1] + radius * np.cos(theta)
            
            # Simulate RSSI measurement (with noise)
            # Quick haversine for simulation truth:
            R = 6371000
            dlat = np.radians(actual_tag[0] - drone_lat)
            dlon = np.radians(actual_tag[1] - drone_lon)
            a = np.sin(dlat/2)**2 + np.cos(np.radians(drone_lat)) * np.cos(np.radians(actual_tag[0])) * np.sin(dlon/2)**2
            c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
            true_dist = R * c
            
            # Expected RSSI + Noise (TxPower=-45, n=2.5)
            rssi = -45 - 10 * 2.5 * np.log10(max(true_dist, 1)) + np.random.normal(0, 2.0)
            
            print(f"Drone: ({drone_lat:.4f}, {drone_lon:.4f}) | RSSI: {rssi:.1f} | Dist: {true_dist:.1f}m")
            
            # Update Localizer
            localizer.update(drone_lat, drone_lon, rssi)
            
            # Visualize
            localizer.visualization(actual_tag_pos=actual_tag, drone_pos=(drone_lat, drone_lon))
            
    except KeyboardInterrupt:
        pass
    lat ,long = localizer.estimate()
    print(f"Estimated Position: ({lat:.4f}, {long:.4f})")
    
    print("Done!")
    plt.show(block=True)
    