"""
Model Predictive Controller for Quadcopter
Implements MPC using CVXPY for trajectory tracking
"""

import numpy as np
import cvxpy as cp
from scipy.linalg import block_diag


class MPCController:
    """
    Model Predictive Controller with constraint handling
    Uses convex optimization for real-time control
    """
    
    def __init__(self, dynamics, dt, horizon=10):
        """
        Initialize MPC controller
        
        Args:
            dynamics: QuadcopterDynamics object
            dt: time step for discretization
            horizon: prediction horizon (number of steps)
        """
        self.dynamics = dynamics
        self.dt = dt
        self.horizon = horizon
        
        # Get linearized model around hover
        state_eq, control_eq = dynamics.get_hover_equilibrium()
        A_cont, B_cont = dynamics.linearize(state_eq, control_eq)
        
        # Discretize using zero-order hold
        self.A, self.B = self.discretize_model(A_cont, B_cont, dt)
        
        # State and control dimensions
        self.nx = 12  # state dimension
        self.nu = 4   # control dimension
        
        # Cost matrices (tune these for performance)
        # Position errors are heavily penalized
        Q = np.diag([100, 100, 100,  # x, y, z position
                     10, 10, 10,      # vx, vy, vz velocity
                     50, 50, 20,      # phi, theta, psi angles
                     1, 1, 1])        # p, q, r angular rates
        
        # Control effort penalty
        R = np.diag([0.1, 10, 10, 10])  # thrust, tau_phi, tau_theta, tau_psi
        
        # Build augmented cost matrices for horizon
        self.Q_bar = block_diag(*[Q for _ in range(horizon)])
        self.R_bar = block_diag(*[R for _ in range(horizon)])
        
        # Constraints
        self.thrust_min = 0.0
        self.thrust_max = 15.0  # N
        self.torque_max = 0.5   # N*m
        
        self.angle_max = np.deg2rad(30)  # Max 30 degree tilt
        
    def discretize_model(self, A, B, dt):
        """
        Discretize continuous-time model using matrix exponential
        
        Returns:
            A_d, B_d: discrete-time matrices
        """
        nx = A.shape[0]
        nu = B.shape[1]
        
        # Using first-order approximation (good for small dt)
        A_d = np.eye(nx) + A * dt
        B_d = B * dt
        
        return A_d, B_d
    
    def solve(self, state_current, state_ref_traj):
        """
        Solve MPC optimization problem
        
        Args:
            state_current: current state [12,]
            state_ref_traj: reference trajectory [horizon+1, 12]
            
        Returns:
            u_opt: optimal control sequence [horizon, 4]
            x_pred: predicted state trajectory [horizon+1, 12]
        """
        # Decision variables
        x = cp.Variable((self.horizon + 1, self.nx))
        u = cp.Variable((self.horizon, self.nu))
        
        # Cost function
        cost = 0
        constraints = []
        
        # Initial condition constraint
        constraints.append(x[0] == state_current)
        
        for k in range(self.horizon):
            # Tracking cost
            state_error = x[k] - state_ref_traj[k]
            control = u[k]
            
            cost += cp.quad_form(state_error, np.eye(self.nx) * 10)
            cost += cp.quad_form(control, np.eye(self.nu) * 0.1)
            
            # Dynamics constraints
            constraints.append(x[k+1] == self.A @ x[k] + self.B @ u[k])
            
            # Control constraints
            constraints.append(u[k, 0] >= self.thrust_min)
            constraints.append(u[k, 0] <= self.thrust_max)
            constraints.append(u[k, 1] >= -self.torque_max)
            constraints.append(u[k, 1] <= self.torque_max)
            constraints.append(u[k, 2] >= -self.torque_max)
            constraints.append(u[k, 2] <= self.torque_max)
            constraints.append(u[k, 3] >= -self.torque_max)
            constraints.append(u[k, 3] <= self.torque_max)
            
            # State constraints (angle limits)
            constraints.append(x[k, 6] >= -self.angle_max)  # phi
            constraints.append(x[k, 6] <= self.angle_max)
            constraints.append(x[k, 7] >= -self.angle_max)  # theta
            constraints.append(x[k, 7] <= self.angle_max)
        
        # Terminal cost
        terminal_error = x[self.horizon] - state_ref_traj[self.horizon]
        cost += cp.quad_form(terminal_error, np.eye(self.nx) * 100)
        
        # Solve optimization problem
        problem = cp.Problem(cp.Minimize(cost), constraints)
        
        try:
            problem.solve(solver=cp.OSQP, verbose=False, warm_start=True)
            
            if problem.status not in ["optimal", "optimal_inaccurate"]:
                print(f"Warning: Solver status: {problem.status}")
                # Return hover control if solver fails
                u_hover = np.array([self.dynamics.m * self.dynamics.g, 0, 0, 0])
                u_opt = np.tile(u_hover, (self.horizon, 1))
                x_pred = np.tile(state_current, (self.horizon + 1, 1))
                return u_opt, x_pred
            
            u_opt = u.value
            x_pred = x.value
            
            return u_opt, x_pred
            
        except Exception as e:
            print(f"Solver error: {e}")
            # Return safe hover control
            u_hover = np.array([self.dynamics.m * self.dynamics.g, 0, 0, 0])
            u_opt = np.tile(u_hover, (self.horizon, 1))
            x_pred = np.tile(state_current, (self.horizon + 1, 1))
            return u_opt, x_pred
    
    def compute_control(self, state_current, state_reference):
        """
        Compute control action for current state
        
        Args:
            state_current: current state
            state_reference: desired reference trajectory
            
        Returns:
            u: control input to apply
        """
        # Create reference trajectory by repeating reference
        state_ref_traj = np.tile(state_reference, (self.horizon + 1, 1))
        
        # Solve MPC
        u_sequence, x_pred = self.solve(state_current, state_ref_traj)
        
        # Return first control action (receding horizon)
        return u_sequence[0], u_sequence, x_pred
