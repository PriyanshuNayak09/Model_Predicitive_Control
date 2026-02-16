"""
Quadcopter Dynamics Model
Implements the full nonlinear 12-state dynamics for a quadcopter
State: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
where:
    - x, y, z: Position in inertial frame
    - vx, vy, vz: Velocity in inertial frame
    - phi, theta, psi: Roll, pitch, yaw (Euler angles)
    - p, q, r: Angular velocities in body frame
"""

import numpy as np
from scipy.integrate import odeint


class QuadcopterDynamics:
    """Quadcopter dynamics with realistic physical parameters"""
    
    def __init__(self):
        # Physical parameters (realistic for a small quadcopter)
        self.m = 0.5  # Mass (kg)
        self.g = 9.81  # Gravity (m/s^2)
        self.L = 0.25  # Arm length (m)
        
        # Moments of inertia (kg*m^2)
        self.Ixx = 5e-3
        self.Iyy = 5e-3
        self.Izz = 9e-3
        
        # Drag coefficients
        self.kd = 0.25
        
    def state_derivative(self, state, t, controls):
        """
        Compute state derivative
        
        Args:
            state: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
            t: time
            controls: [thrust, tau_phi, tau_theta, tau_psi]
        
        Returns:
            state_dot: derivative of state
        """
        # Unpack state
        x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = state
        
        # Unpack controls
        thrust, tau_phi, tau_theta, tau_psi = controls
        
        # Rotation matrix from body to inertial frame
        c_phi = np.cos(phi)
        s_phi = np.sin(phi)
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)
        c_psi = np.cos(psi)
        s_psi = np.sin(psi)
        
        # Position derivatives
        x_dot = vx
        y_dot = vy
        z_dot = vz
        
        # Velocity derivatives (Newton's second law in inertial frame)
        # Thrust force in body frame is [0, 0, thrust]
        # Transform to inertial frame
        vx_dot = (c_psi * s_theta * c_phi + s_psi * s_phi) * thrust / self.m
        vy_dot = (s_psi * s_theta * c_phi - c_psi * s_phi) * thrust / self.m
        vz_dot = (c_theta * c_phi) * thrust / self.m - self.g
        
        # Euler angle derivatives
        phi_dot = p + s_phi * np.tan(theta) * q + c_phi * np.tan(theta) * r
        theta_dot = c_phi * q - s_phi * r
        psi_dot = (s_phi / c_theta) * q + (c_phi / c_theta) * r
        
        # Angular acceleration (Euler's equations)
        p_dot = (tau_phi + (self.Iyy - self.Izz) * q * r) / self.Ixx
        q_dot = (tau_theta + (self.Izz - self.Ixx) * p * r) / self.Iyy
        r_dot = (tau_psi + (self.Ixx - self.Iyy) * p * q) / self.Izz
        
        return np.array([x_dot, y_dot, z_dot, vx_dot, vy_dot, vz_dot,
                        phi_dot, theta_dot, psi_dot, p_dot, q_dot, r_dot])
    
    def simulate_step(self, state, controls, dt):
        """
        Simulate one time step using numerical integration
        
        Args:
            state: current state vector
            controls: control inputs
            dt: time step
            
        Returns:
            next_state: state after dt seconds
        """
        t = np.array([0, dt])
        sol = odeint(self.state_derivative, state, t, args=(controls,))
        return sol[-1]
    
    def linearize(self, state_eq, control_eq):
        """
        Linearize dynamics around equilibrium point
        Useful for MPC with linear model
        
        Args:
            state_eq: equilibrium state (hovering)
            control_eq: equilibrium control (hovering thrust)
            
        Returns:
            A, B: state-space matrices
        """
        # For hover: phi=theta=psi=0, p=q=r=0, vx=vy=vz=0
        # Only position can vary
        
        # Simplified linearized dynamics around hover
        A = np.zeros((12, 12))
        
        # Position derivatives
        A[0, 3] = 1  # x_dot = vx
        A[1, 4] = 1  # y_dot = vy
        A[2, 5] = 1  # z_dot = vz
        
        # Velocity derivatives (linearized around hover)
        A[3, 7] = self.g  # vx affected by theta
        A[4, 6] = -self.g  # vy affected by phi
        
        # Angle derivatives
        A[6, 9] = 1   # phi_dot = p
        A[7, 10] = 1  # theta_dot = q
        A[8, 11] = 1  # psi_dot = r
        
        # Control matrix
        B = np.zeros((12, 4))
        B[5, 0] = 1/self.m  # thrust affects vz
        B[9, 1] = 1/self.Ixx  # tau_phi affects p
        B[10, 2] = 1/self.Iyy  # tau_theta affects q
        B[11, 3] = 1/self.Izz  # tau_psi affects r
        
        return A, B
    
    def get_hover_equilibrium(self, z_desired=0):
        """
        Get equilibrium state and control for hovering at height z_desired
        
        Returns:
            state_eq, control_eq
        """
        state_eq = np.zeros(12)
        state_eq[2] = z_desired  # z position
        
        control_eq = np.array([self.m * self.g, 0, 0, 0])  # Thrust to counteract gravity
        
        return state_eq, control_eq
