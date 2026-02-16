"""
Quadcopter MPC - Main Execution Script
Demonstrates Model Predictive Control for trajectory tracking
"""

import numpy as np
import sys
import os

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from quadcopter_dynamics import QuadcopterDynamics
from mpc_controller import MPCController
from trajectory_generator import TrajectoryGenerator
from simulator import Simulator


def main():
    """Main execution function"""
    
    print("="*60)
    print("QUADCOPTER MODEL PREDICTIVE CONTROL SIMULATION")
    print("="*60)
    print()
    
    # ==================== Configuration ====================
    dt = 0.05           # Time step (20 Hz control rate)
    horizon = 15        # MPC prediction horizon
    duration = 30.0     # Simulation duration
    
    # Initialize dynamics
    print("Initializing quadcopter dynamics...")
    dynamics = QuadcopterDynamics()
    
    # Initialize MPC controller
    print("Initializing MPC controller...")
    print(f"  - Time step: {dt}s ({1/dt:.0f} Hz)")
    print(f"  - Prediction horizon: {horizon} steps ({horizon*dt:.2f}s)")
    controller = MPCController(dynamics, dt, horizon=horizon)
    
    # ==================== Trajectory Selection ====================
    print("\n" + "="*60)
    print("Available Trajectories:")
    print("="*60)
    print("1. Circle")
    print("2. Figure-8")
    print("3. Helix (Spiral)")
    print("4. Hover")
    print("5. Waypoint")
    
    trajectory_choice = input("\nSelect trajectory (1-5) [default=1]: ").strip()
    if not trajectory_choice:
        trajectory_choice = '1'
    
    # Define trajectory function
    if trajectory_choice == '1':
        print("\n→ Selected: Circle Trajectory")
        trajectory_fn = lambda t: TrajectoryGenerator.circle_trajectory(
            t, radius=3.0, height=5.0, period=15.0)
        traj_name = "circle"
        
    elif trajectory_choice == '2':
        print("\n→ Selected: Figure-8 Trajectory")
        trajectory_fn = lambda t: TrajectoryGenerator.figure_eight_trajectory(
            t, scale=3.0, height=5.0, period=20.0)
        traj_name = "figure8"
        
    elif trajectory_choice == '3':
        print("\n→ Selected: Helix Trajectory")
        trajectory_fn = lambda t: TrajectoryGenerator.helix_trajectory(
            t, radius=2.5, height_rate=0.3, period=12.0)
        traj_name = "helix"
        
    elif trajectory_choice == '4':
        print("\n→ Selected: Hover Trajectory")
        trajectory_fn = lambda t: TrajectoryGenerator.hover_trajectory([0, 0, 5])
        traj_name = "hover"
        
    elif trajectory_choice == '5':
        print("\n→ Selected: Waypoint Trajectory")
        waypoints = [[0, 0, 2], [5, 0, 4], [5, 5, 6], [0, 5, 4], [0, 0, 2]]
        times = [0, 7, 14, 21, 28]
        trajectory_fn = lambda t: TrajectoryGenerator.waypoint_trajectory(
            t, waypoints, times)
        traj_name = "waypoint"
        
    else:
        print("\nInvalid choice, using Circle trajectory")
        trajectory_fn = lambda t: TrajectoryGenerator.circle_trajectory(
            t, radius=3.0, height=5.0, period=15.0)
        traj_name = "circle"
    
    # ==================== Initial Conditions ====================
    print("\n" + "="*60)
    print("Setting Initial Conditions")
    print("="*60)
    
    # Start at origin with slight offset
    initial_state = np.zeros(12)
    initial_state[0] = 0.5   # x offset
    initial_state[2] = 0.5   # z offset (start low)
    
    print(f"Initial position: [{initial_state[0]:.2f}, "
          f"{initial_state[1]:.2f}, {initial_state[2]:.2f}] m")
    
    # ==================== Run Simulation ====================
    print("\n" + "="*60)
    print("Running Simulation")
    print("="*60)
    
    # Create graph directory
    graph_dir = 'graphs'
    os.makedirs(graph_dir, exist_ok=True)
    
    # Initialize simulator
    simulator = Simulator(dynamics, controller, trajectory_fn, dt)
    
    # Run simulation
    results = simulator.run(
        initial_state=initial_state,
        duration=duration,
        save_graphs=True,
        graph_dir=graph_dir
    )
    
    print("\n" + "="*60)
    print("SIMULATION COMPLETE")
    print("="*60)
    print(f"\nAll graphs saved to '{graph_dir}/' directory")
    print("\nGenerated files:")
    print("  • 3d_trajectory.png")
    print("  • position_tracking.png")
    print("  • velocity_tracking.png")
    print("  • attitude_angles.png")
    print("  • angular_rates.png")
    print("  • control_inputs.png")
    print("  • tracking_error.png")
    print("  • trajectory_top_view.png")
    
    return results


if __name__ == "__main__":
    try:
        results = main()
        print("\n✓ Simulation completed successfully!")
    except KeyboardInterrupt:
        print("\n\n⚠ Simulation interrupted by user")
    except Exception as e:
        print(f"\n✗ Error during simulation: {e}")
        import traceback
        traceback.print_exc()
