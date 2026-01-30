"""
Gaussian Process Belief Map for Intelligent Scanning
=====================================================
This module maintains a probabilistic model of where BLE tags are likely
located based on RSSI observations.

Key concepts:
- GP predicts RSSI values at unvisited locations
- UCB (Upper Confidence Bound) balances exploration vs exploitation
- The belief map guides the drone to the most promising areas
"""

import math
import numpy as np


class GaussianProcess:
    """
    Simple Gaussian Process for spatial RSSI prediction.
    
    How it works:
    1. You give it observations: (x, y) -> rssi
    2. It learns the spatial correlation
    3. It predicts rssi at new locations + uncertainty
    
    The key insight: nearby points have similar RSSI values.
    This is encoded in the "kernel" function.
    """
    
    def __init__(self, length_scale=50.0, signal_variance=400.0, noise_variance=16.0):
        """
        Args:
            length_scale: How far spatial correlation extends (meters)
                         50m allows correlation across typical exploration area
            signal_variance: Expected variance in RSSI values (std=20 dBm range)
            noise_variance: Measurement noise (4 dBm std -> 16 variance)
        """
        self.length_scale = length_scale
        self.signal_variance = signal_variance
        self.noise_variance = noise_variance
        
        # Training data
        self.X_train = None  # Nx2 array of (x, y) positions
        self.y_train = None  # N array of RSSI values
        
        # Cached matrices for prediction
        self._K_inv = None
        self._alpha = None
    
    def kernel(self, X1, X2):
        """
        Matern 3/2 kernel - measures similarity between points.
        
        Returns high values when points are close, low when far.
        Matern is more realistic than RBF for physical signals.
        """
        # Calculate pairwise distances
        # X1: (n1, 2), X2: (n2, 2)
        X1 = np.atleast_2d(X1)
        X2 = np.atleast_2d(X2)
        
        # Squared Euclidean distance
        diff = X1[:, np.newaxis, :] - X2[np.newaxis, :, :]
        dist = np.sqrt(np.sum(diff ** 2, axis=2))
        
        # Matern 3/2 kernel
        sqrt3 = math.sqrt(3)
        scaled_dist = sqrt3 * dist / self.length_scale
        K = self.signal_variance * (1 + scaled_dist) * np.exp(-scaled_dist)
        
        return K
    
    def fit(self, X, y):
        """
        Fit the GP to observed data.
        
        Args:
            X: Array of positions, shape (n, 2) - each row is (x, y)
            y: Array of RSSI values, shape (n,)
        """
        self.X_train = np.atleast_2d(X)
        self.y_train = np.atleast_1d(y)
        
        n = len(self.y_train)
        
        # Compute kernel matrix K(X, X)
        K = self.kernel(self.X_train, self.X_train)
        
        # Add noise to diagonal
        K += self.noise_variance * np.eye(n)
        
        # Compute inverse (used for predictions)
        # Adding small jitter for numerical stability
        K += 1e-6 * np.eye(n)
        
        try:
            L = np.linalg.cholesky(K)
            self._alpha = np.linalg.solve(L.T, np.linalg.solve(L, self.y_train))
            self._K_inv = np.linalg.solve(L.T, np.linalg.solve(L, np.eye(n)))
        except np.linalg.LinAlgError:
            # Fallback to direct inverse if Cholesky fails
            self._K_inv = np.linalg.inv(K)
            self._alpha = self._K_inv @ self.y_train
    
    def predict(self, X_new):
        """
        Predict RSSI at new locations.
        
        Args:
            X_new: Array of positions to predict, shape (m, 2)
        
        Returns:
            mean: Predicted RSSI values, shape (m,)
            std: Prediction uncertainty (std dev), shape (m,)
        """
        if self.X_train is None:
            # No training data - return prior
            m = len(np.atleast_2d(X_new))
            return np.full(m, -75.0), np.full(m, 20.0)  # Prior: -75 dBm ± 20
        
        X_new = np.atleast_2d(X_new)
        
        # K(X_new, X_train)
        K_s = self.kernel(X_new, self.X_train)
        
        # K(X_new, X_new)
        K_ss = self.kernel(X_new, X_new)
        
        # Mean prediction
        mean = K_s @ self._alpha
        
        # Variance prediction
        v = K_s @ self._K_inv @ K_s.T
        var = np.diag(K_ss) - np.diag(v)
        var = np.maximum(var, 1e-6)  # Ensure positive
        std = np.sqrt(var)
        
        return mean, std
    
    def get_observation_count(self):
        """Return number of observations."""
        if self.X_train is None:
            return 0
        return len(self.y_train)


class BeliefMap:
    """
    Manages the GP and provides the exploration strategy.
    
    This is the "brain" that decides where to explore next.
    It uses UCB (Upper Confidence Bound) to balance:
    - Exploitation: Go where RSSI is predicted to be high
    - Exploration: Go where we're uncertain
    """
    
    def __init__(self, bounds, grid_resolution=5.0, beta=2.0):
        """
        Args:
            bounds: Dict with 'x_min', 'x_max', 'y_min', 'y_max' in meters
            grid_resolution: Spacing between candidate points (meters)
            beta: Exploration weight. Higher = more exploration.
        """
        self.bounds = bounds
        self.grid_resolution = grid_resolution
        self.beta = beta
        
        self.gp = GaussianProcess()
        
        # Generate candidate grid
        self._generate_candidates()
    
    def _generate_candidates(self):
        """Generate grid of candidate positions for UCB evaluation."""
        x_range = np.arange(
            self.bounds['x_min'], 
            self.bounds['x_max'], 
            self.grid_resolution
        )
        y_range = np.arange(
            self.bounds['y_min'], 
            self.bounds['y_max'], 
            self.grid_resolution
        )
        
        # Create meshgrid
        xx, yy = np.meshgrid(x_range, y_range)
        self.candidates = np.column_stack([xx.ravel(), yy.ravel()])
    
    def update(self, x, y, rssi):
        """
        Add a new observation and refit the GP.
        
        Args:
            x, y: Position in local coordinates (meters)
            rssi: Observed RSSI value (dBm)
        """
        new_point = np.array([[x, y]])
        new_rssi = np.array([rssi])
        
        if self.gp.X_train is None:
            self.gp.fit(new_point, new_rssi)
        else:
            X = np.vstack([self.gp.X_train, new_point])
            y = np.concatenate([self.gp.y_train, new_rssi])
            self.gp.fit(X, y)
    
    def get_next_target(self):
        """
        Find the best next position to explore using UCB + coverage bonus.
        
        UCB = mean + beta * std + gamma * coverage_bonus
        
        - High mean: We expect good signal there (exploitation)
        - High std: We're uncertain (exploration)
        - High coverage_bonus: Far from visited locations (coverage)
        
        Returns:
            (x, y): Best position to explore
            ucb_value: UCB score at that position
        """
        if len(self.candidates) == 0:
            return None, None
        
        # Predict mean and std at all candidates
        mean, std = self.gp.predict(self.candidates)
        
        # Compute basic UCB
        ucb = mean + self.beta * std
        
        # Add coverage bonus: reward being far from all observations
        if self.gp.X_train is not None and len(self.gp.X_train) > 0:
            # For each candidate, find min distance to any observation
            min_distances = np.zeros(len(self.candidates))
            for i, cand in enumerate(self.candidates):
                dists = np.sqrt(np.sum((self.gp.X_train - cand) ** 2, axis=1))
                min_distances[i] = np.min(dists)
            
            # Normalize and scale coverage bonus (gamma = 0.5)
            coverage_bonus = min_distances / 50.0  # Normalize by ~50m
            ucb = ucb + 0.5 * coverage_bonus * np.abs(np.mean(mean))  # Scale by mean magnitude
        
        # Find best candidate
        best_idx = np.argmax(ucb)
        best_pos = self.candidates[best_idx]
        best_ucb = ucb[best_idx]
        
        return tuple(best_pos), best_ucb
    
    def get_belief_grid(self):
        """
        Get the full predicted map for visualization.
        
        Returns:
            candidates: (N, 2) array of positions
            mean: (N,) predicted RSSI
            std: (N,) uncertainty
            ucb: (N,) UCB values
        """
        mean, std = self.gp.predict(self.candidates)
        ucb = mean + self.beta * std
        
        return self.candidates, mean, std, ucb


class AdaptivePlanner:
    """
    Top-level planner that manages the belief map and generates waypoints.
    
    This is what Layer 2 uses. It:
    1. Receives RSSI observations
    2. Updates the belief map
    3. Provides the next GPS waypoint
    """
    
    def __init__(self, origin_lat, origin_lon, area_size=100.0):
        """
        Args:
            origin_lat, origin_lon: GPS origin (Gazebo world origin)
            area_size: Size of the search area (meters, square)
        """
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        
        # Define search bounds (centered on origin, extending in all directions)
        half = area_size / 2
        bounds = {
            'x_min': -half, 'x_max': half,
            'y_min': -half, 'y_max': half
        }
        
        self.belief_map = BeliefMap(bounds)
        
        # Track discovered tags
        self.discovered_tags = set()
        self.observation_count = 0
    
    def gps_to_local(self, lat, lon):
        """Convert GPS to local (x, y) meters."""
        R = 6371000
        x = R * math.radians(lon - self.origin_lon) * math.cos(math.radians(self.origin_lat))
        y = R * math.radians(lat - self.origin_lat)
        return x, y
    
    def local_to_gps(self, x, y):
        """Convert local (x, y) meters to GPS."""
        R = 6371000
        lat = self.origin_lat + math.degrees(y / R)
        lon = self.origin_lon + math.degrees(x / (R * math.cos(math.radians(self.origin_lat))))
        return lat, lon
    
    def add_observation(self, lat, lon, rssi):
        """
        Add an RSSI observation from the drone.
        
        Args:
            lat, lon: Drone's GPS position
            rssi: Strongest RSSI reading (or average)
        """
        x, y = self.gps_to_local(lat, lon)
        self.belief_map.update(x, y, rssi)
        self.observation_count += 1
    
    def get_next_waypoint(self):
        """
        Get the next GPS waypoint to explore.
        
        Returns:
            (lat, lon): GPS coordinates
            score: UCB score (for logging)
        """
        local_target, score = self.belief_map.get_next_target()
        
        if local_target is None:
            return None, None
        
        lat, lon = self.local_to_gps(local_target[0], local_target[1])
        return (lat, lon), score
    
    def get_statistics(self):
        """Get planner statistics."""
        return {
            'observations': self.observation_count,
            'gp_fitted': self.belief_map.gp.get_observation_count() > 0
        }


# =============================================================================
# TEST
# =============================================================================

if __name__ == "__main__":
    print("=" * 50)
    print("Gaussian Process Belief Map Test")
    print("=" * 50)
    
    # Create planner centered at SITL origin
    planner = AdaptivePlanner(
        origin_lat=47.397971,
        origin_lon=8.546164,
        area_size=100.0
    )
    
    # Simulate some observations
    observations = [
        (47.3978, 8.5455, -70),  # Weak signal
        (47.3979, 8.5456, -60),  # Medium signal
        (47.3980, 8.5457, -50),  # Strong signal
    ]
    
    print("\nAdding observations:")
    for lat, lon, rssi in observations:
        planner.add_observation(lat, lon, rssi)
        print(f"  ({lat}, {lon}) -> {rssi} dBm")
    
    # Get next waypoint
    waypoint, score = planner.get_next_waypoint()
    print(f"\nNext waypoint: ({waypoint[0]:.6f}, {waypoint[1]:.6f})")
    print(f"UCB score: {score:.1f}")
    
    # Test GP directly
    print("\n--- GP Direct Test ---")
    gp = GaussianProcess()
    
    X = np.array([[0, 0], [10, 0], [20, 0]])
    y = np.array([-60, -50, -70])
    
    gp.fit(X, y)
    
    X_test = np.array([[5, 0], [15, 0], [25, 0]])
    mean, std = gp.predict(X_test)
    
    print("Predictions:")
    for i, (m, s) in enumerate(zip(mean, std)):
        print(f"  ({X_test[i, 0]}, {X_test[i, 1]}): {m:.1f} ± {s:.1f} dBm")
    
    print("\n" + "=" * 50)
