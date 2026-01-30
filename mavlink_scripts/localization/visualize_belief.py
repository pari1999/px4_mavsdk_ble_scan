"""
Visualization tools for the Gaussian Process belief map.
Creates heatmaps showing predicted RSSI and uncertainty.

Usage:
    python3 visualize_belief.py
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from gaussian_process import AdaptivePlanner


def visualize_belief_map(planner, tag_positions=None, drone_path=None, 
                         save_path=None, show=True):
    """
    Create visualization of the belief map.
    
    Args:
        planner: AdaptivePlanner instance with observations
        tag_positions: List of (x, y) tag positions (optional)
        drone_path: List of (x, y) drone positions (optional)
        save_path: Path to save the figure (optional)
        show: Whether to display the plot
    """
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # Get belief data
    candidates, mean, std, ucb = planner.belief_map.get_belief_grid()
    
    # Reshape for 2D plotting
    x_unique = np.unique(candidates[:, 0])
    y_unique = np.unique(candidates[:, 1])
    nx, ny = len(x_unique), len(y_unique)
    
    mean_grid = mean.reshape(ny, nx)
    std_grid = std.reshape(ny, nx)
    ucb_grid = ucb.reshape(ny, nx)
    
    extent = [x_unique.min(), x_unique.max(), y_unique.min(), y_unique.max()]
    
    # Plot 1: Mean RSSI
    ax1 = axes[0, 0]
    im1 = ax1.imshow(mean_grid, extent=extent, origin='lower', 
                     cmap='RdYlGn', aspect='equal')
    ax1.set_title('Predicted RSSI (Mean)')
    ax1.set_xlabel('X (meters)')
    ax1.set_ylabel('Y (meters)')
    plt.colorbar(im1, ax=ax1, label='dBm')
    
    # Plot 2: Uncertainty
    ax2 = axes[0, 1]
    im2 = ax2.imshow(std_grid, extent=extent, origin='lower', 
                     cmap='Purples', aspect='equal')
    ax2.set_title('Uncertainty (Std Dev)')
    ax2.set_xlabel('X (meters)')
    ax2.set_ylabel('Y (meters)')
    plt.colorbar(im2, ax=ax2, label='dBm')
    
    # Plot 3: UCB
    ax3 = axes[1, 0]
    im3 = ax3.imshow(ucb_grid, extent=extent, origin='lower', 
                     cmap='hot', aspect='equal')
    ax3.set_title('UCB Score (Mean + 2×Std)')
    ax3.set_xlabel('X (meters)')
    ax3.set_ylabel('Y (meters)')
    plt.colorbar(im3, ax=ax3, label='Score')
    
    # Get next target
    next_target, _ = planner.belief_map.get_next_target()
    if next_target:
        ax3.scatter(next_target[0], next_target[1], 
                   marker='*', s=200, c='cyan', edgecolors='black',
                   label='Next Target', zorder=10)
        ax3.legend()
    
    # Plot 4: Observations + Tags
    ax4 = axes[1, 1]
    ax4.set_xlim(extent[0], extent[1])
    ax4.set_ylim(extent[2], extent[3])
    ax4.set_aspect('equal')
    ax4.set_title('Observations & Tags')
    ax4.set_xlabel('X (meters)')
    ax4.set_ylabel('Y (meters)')
    ax4.grid(True, alpha=0.3)
    
    # Plot observations
    if planner.belief_map.gp.X_train is not None:
        obs = planner.belief_map.gp.X_train
        colors = planner.belief_map.gp.y_train
        sc = ax4.scatter(obs[:, 0], obs[:, 1], c=colors, cmap='RdYlGn', 
                        s=30, edgecolors='black', label='Observations')
        plt.colorbar(sc, ax=ax4, label='RSSI (dBm)')
    
    # Plot tag positions
    if tag_positions:
        for x, y in tag_positions:
            circle = Circle((x, y), radius=2, fill=False, 
                           color='red', linewidth=2)
            ax4.add_patch(circle)
        ax4.scatter([], [], marker='o', s=100, facecolors='none', 
                   edgecolors='red', linewidths=2, label='Tags')
    
    # Plot drone path
    if drone_path and len(drone_path) > 1:
        path = np.array(drone_path)
        ax4.plot(path[:, 0], path[:, 1], 'b-', linewidth=1, alpha=0.5)
        ax4.scatter(path[-1, 0], path[-1, 1], marker='^', s=100, 
                   c='blue', label='Drone')
    
    ax4.legend(loc='upper right')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Saved to {save_path}")
    
    if show:
        plt.show()
    
    return fig


def demo_visualization():
    """Run a demo showing the belief map evolving."""
    print("=" * 50)
    print("Belief Map Visualization Demo")
    print("=" * 50)
    
    # Create planner
    planner = AdaptivePlanner(
        origin_lat=47.397971,
        origin_lon=8.546164,
        area_size=80.0
    )
    
    # Simulated tag positions (local coordinates)
    tag_positions = [
        (-20, -10),
        (-25, -15),
        (10, 20),
        (15, 25),
        (-30, 30),
    ]
    
    # Simulate observations as if flying
    observations = [
        (-40, -40, -85),  # Far from tags, weak
        (-30, -30, -80),
        (-25, -20, -65),  # Getting closer to cluster 1
        (-22, -12, -55),  # Near tag!
        (0, 0, -75),      # Moving across
        (5, 10, -70),
        (12, 22, -50),    # Near cluster 2
        (-20, 20, -72),
        (-28, 28, -58),   # Near isolated tag
    ]
    
    drone_path = []
    
    print("\nAdding observations:")
    for x, y, rssi in observations:
        # Convert local to GPS for the API
        lat, lon = planner.local_to_gps(x, y)
        planner.add_observation(lat, lon, rssi)
        drone_path.append((x, y))
        print(f"  ({x:3.0f}, {y:3.0f}) -> {rssi} dBm")
    
    # Visualize
    print("\nGenerating visualization...")
    visualize_belief_map(
        planner, 
        tag_positions=tag_positions,
        drone_path=drone_path,
        save_path="belief_map.png",
        show=True
    )


if __name__ == "__main__":
    demo_visualization()
