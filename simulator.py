"""
Simulation Environment
Runs closed-loop simulation with MPC controller and quadcopter dynamics
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import os


class Simulator:
    """Closed-loop simulation environment"""
    
    def __init__(self, dynamics, controller, trajectory_fn, dt=0.05):
        """
        Initialize simulator
        
        Args:
            dynamics: QuadcopterDynamics object
            controller: MPCController object
            trajectory_fn: function that takes time and returns reference state
            dt: simulation time step
        """
        self.dynamics = dynamics
        self.controller = controller
        self.trajectory_fn = trajectory_fn
        self.dt = dt
        
        # Data storage
        self.time_history = []
        self.state_history = []
        self.control_history = []
        self.reference_history = []
        self.predicted_trajectories = []
        
    def run(self, initial_state, duration, save_graphs=True, graph_dir='graphs'):
        """
        Run closed-loop simulation
        
        Args:
            initial_state: initial state [12,]
            duration: simulation duration (seconds)
            save_graphs: whether to save plots
            graph_dir: directory to save graphs
            
        Returns:
            Dictionary with simulation results
        """
        print("Starting simulation...")
        
        # Initialize
        state = initial_state.copy()
        t = 0
        steps = int(duration / self.dt)
        
        # Create graph directory if it doesn't exist
        if save_graphs and not os.path.exists(graph_dir):
            os.makedirs(graph_dir)
        
        for step in range(steps):
            # Get reference state
            ref_state = self.trajectory_fn(t)
            
            # Compute control using MPC
            try:
                control, u_seq, x_pred = self.controller.compute_control(state, ref_state)
            except Exception as e:
                print(f"MPC failed at t={t:.2f}: {e}")
                control = np.array([self.dynamics.m * self.dynamics.g, 0, 0, 0])
                x_pred = None
            
            # Store data
            self.time_history.append(t)
            self.state_history.append(state.copy())
            self.control_history.append(control.copy())
            self.reference_history.append(ref_state.copy())
            if x_pred is not None:
                self.predicted_trajectories.append(x_pred.copy())
            
            # Simulate dynamics
            state = self.dynamics.simulate_step(state, control, self.dt)
            
            # Progress update
            if step % 20 == 0:
                error = np.linalg.norm(state[0:3] - ref_state[0:3])
                print(f"Time: {t:.2f}s, Position Error: {error:.4f}m")
            
            t += self.dt
        
        # Convert to numpy arrays
        self.time_history = np.array(self.time_history)
        self.state_history = np.array(self.state_history)
        self.control_history = np.array(self.control_history)
        self.reference_history = np.array(self.reference_history)
        
        print(f"\nSimulation complete! Total time: {duration}s")
        
        # Generate and save plots
        if save_graphs:
            self.plot_results(graph_dir)
        
        return {
            'time': self.time_history,
            'states': self.state_history,
            'controls': self.control_history,
            'references': self.reference_history
        }
    
    def plot_results(self, save_dir='graphs'):
        """Generate comprehensive plots of simulation results"""
        
        print("\nGenerating plots...")
        
        # 1. 3D Trajectory
        fig = plt.figure(figsize=(12, 9))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot actual trajectory
        ax.plot(self.state_history[:, 0], 
                self.state_history[:, 1], 
                self.state_history[:, 2],
                'b-', linewidth=2, label='Actual Trajectory')
        
        # Plot reference trajectory
        ax.plot(self.reference_history[:, 0],
                self.reference_history[:, 1],
                self.reference_history[:, 2],
                'r--', linewidth=2, label='Reference Trajectory')
        
        # Mark start and end
        ax.scatter(self.state_history[0, 0], self.state_history[0, 1], 
                  self.state_history[0, 2], c='g', s=100, marker='o', label='Start')
        ax.scatter(self.state_history[-1, 0], self.state_history[-1, 1],
                  self.state_history[-1, 2], c='r', s=100, marker='x', label='End')
        
        ax.set_xlabel('X Position (m)', fontsize=12)
        ax.set_ylabel('Y Position (m)', fontsize=12)
        ax.set_zlabel('Z Position (m)', fontsize=12)
        ax.set_title('3D Trajectory Tracking', fontsize=14, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(f'{save_dir}/3d_trajectory.png', dpi=300, bbox_inches='tight')
        plt.close()
        print("  ✓ Saved 3d_trajectory.png")
        
        # 2. Position Tracking
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        labels = ['X', 'Y', 'Z']
        
        for i, (ax, label) in enumerate(zip(axes, labels)):
            ax.plot(self.time_history, self.state_history[:, i], 
                   'b-', linewidth=2, label='Actual')
            ax.plot(self.time_history, self.reference_history[:, i],
                   'r--', linewidth=2, label='Reference')
            ax.set_ylabel(f'{label} Position (m)', fontsize=11)
            ax.legend(fontsize=10)
            ax.grid(True, alpha=0.3)
            
        axes[-1].set_xlabel('Time (s)', fontsize=11)
        axes[0].set_title('Position Tracking', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(f'{save_dir}/position_tracking.png', dpi=300, bbox_inches='tight')
        plt.close()
        print("  ✓ Saved position_tracking.png")
        
        # 3. Velocity Tracking
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        labels = ['Vx', 'Vy', 'Vz']
        
        for i, (ax, label) in enumerate(zip(axes, labels)):
            ax.plot(self.time_history, self.state_history[:, i+3],
                   'b-', linewidth=2, label='Actual')
            ax.plot(self.time_history, self.reference_history[:, i+3],
                   'r--', linewidth=2, label='Reference')
            ax.set_ylabel(f'{label} (m/s)', fontsize=11)
            ax.legend(fontsize=10)
            ax.grid(True, alpha=0.3)
            
        axes[-1].set_xlabel('Time (s)', fontsize=11)
        axes[0].set_title('Velocity Tracking', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(f'{save_dir}/velocity_tracking.png', dpi=300, bbox_inches='tight')
        plt.close()
        print("  ✓ Saved velocity_tracking.png")
        
        # 4. Attitude (Euler Angles)
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        labels = ['Roll (φ)', 'Pitch (θ)', 'Yaw (ψ)']
        
        for i, (ax, label) in enumerate(zip(axes, labels)):
            ax.plot(self.time_history, np.rad2deg(self.state_history[:, i+6]),
                   'b-', linewidth=2, label='Actual')
            ax.plot(self.time_history, np.rad2deg(self.reference_history[:, i+6]),
                   'r--', linewidth=2, label='Reference')
            ax.set_ylabel(f'{label} (deg)', fontsize=11)
            ax.legend(fontsize=10)
            ax.grid(True, alpha=0.3)
            
        axes[-1].set_xlabel('Time (s)', fontsize=11)
        axes[0].set_title('Attitude Angles', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(f'{save_dir}/attitude_angles.png', dpi=300, bbox_inches='tight')
        plt.close()
        print("  ✓ Saved attitude_angles.png")
        
        # 5. Angular Rates
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        labels = ['p (roll rate)', 'q (pitch rate)', 'r (yaw rate)']
        
        for i, (ax, label) in enumerate(zip(axes, labels)):
            ax.plot(self.time_history, np.rad2deg(self.state_history[:, i+9]),
                   'b-', linewidth=2)
            ax.set_ylabel(f'{label} (deg/s)', fontsize=11)
            ax.grid(True, alpha=0.3)
            
        axes[-1].set_xlabel('Time (s)', fontsize=11)
        axes[0].set_title('Angular Rates (Body Frame)', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(f'{save_dir}/angular_rates.png', dpi=300, bbox_inches='tight')
        plt.close()
        print("  ✓ Saved angular_rates.png")
        
        # 6. Control Inputs
        fig, axes = plt.subplots(4, 1, figsize=(12, 12))
        labels = ['Thrust (N)', 'Torque φ (N⋅m)', 'Torque θ (N⋅m)', 'Torque ψ (N⋅m)']
        
        for i, (ax, label) in enumerate(zip(axes, labels)):
            ax.plot(self.time_history, self.control_history[:, i],
                   'g-', linewidth=2)
            ax.set_ylabel(label, fontsize=11)
            ax.grid(True, alpha=0.3)
            
            # Add constraint lines
            if i == 0:
                ax.axhline(y=self.controller.thrust_max, color='r', 
                          linestyle='--', alpha=0.5, label='Max')
                ax.axhline(y=self.controller.thrust_min, color='r',
                          linestyle='--', alpha=0.5, label='Min')
            else:
                ax.axhline(y=self.controller.torque_max, color='r',
                          linestyle='--', alpha=0.5, label='Max')
                ax.axhline(y=-self.controller.torque_max, color='r',
                          linestyle='--', alpha=0.5, label='Min')
            ax.legend(fontsize=9)
            
        axes[-1].set_xlabel('Time (s)', fontsize=11)
        axes[0].set_title('Control Inputs', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(f'{save_dir}/control_inputs.png', dpi=300, bbox_inches='tight')
        plt.close()
        print("  ✓ Saved control_inputs.png")
        
        # 7. Tracking Error
        position_error = np.linalg.norm(
            self.state_history[:, 0:3] - self.reference_history[:, 0:3], axis=1)
        
        fig, ax = plt.subplots(figsize=(12, 6))
        ax.plot(self.time_history, position_error, 'b-', linewidth=2)
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Position Error (m)', fontsize=12)
        ax.set_title('Tracking Error Norm', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        # Add statistics
        mean_error = np.mean(position_error)
        max_error = np.max(position_error)
        ax.axhline(y=mean_error, color='r', linestyle='--', 
                  label=f'Mean: {mean_error:.4f}m')
        ax.text(0.02, 0.98, f'Max Error: {max_error:.4f}m', 
               transform=ax.transAxes, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        ax.legend(fontsize=10)
        
        plt.tight_layout()
        plt.savefig(f'{save_dir}/tracking_error.png', dpi=300, bbox_inches='tight')
        plt.close()
        print("  ✓ Saved tracking_error.png")
        
        # 8. X-Y Trajectory (Top View)
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.plot(self.state_history[:, 0], self.state_history[:, 1],
               'b-', linewidth=2, label='Actual')
        ax.plot(self.reference_history[:, 0], self.reference_history[:, 1],
               'r--', linewidth=2, label='Reference')
        ax.scatter(self.state_history[0, 0], self.state_history[0, 1],
                  c='g', s=100, marker='o', label='Start', zorder=5)
        ax.scatter(self.state_history[-1, 0], self.state_history[-1, 1],
                  c='r', s=100, marker='x', label='End', zorder=5)
        
        ax.set_xlabel('X Position (m)', fontsize=12)
        ax.set_ylabel('Y Position (m)', fontsize=12)
        ax.set_title('Trajectory (Top View)', fontsize=14, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        plt.tight_layout()
        plt.savefig(f'{save_dir}/trajectory_top_view.png', dpi=300, bbox_inches='tight')
        plt.close()
        print("  ✓ Saved trajectory_top_view.png")
        
        print(f"\n✓ All plots saved to '{save_dir}/' directory")
        
        # Print summary statistics
        print("\n" + "="*50)
        print("SIMULATION SUMMARY")
        print("="*50)
        print(f"Duration: {self.time_history[-1]:.2f}s")
        print(f"Time steps: {len(self.time_history)}")
        print(f"Mean position error: {mean_error:.4f}m")
        print(f"Max position error: {max_error:.4f}m")
        print(f"Final position error: {position_error[-1]:.4f}m")
        print("="*50)
