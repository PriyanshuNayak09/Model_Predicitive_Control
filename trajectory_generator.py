"""
Trajectory Generator
Generates reference trajectories for the quadcopter to follow
"""

import numpy as np


class TrajectoryGenerator:
    """Generate various reference trajectories"""
    
    @staticmethod
    def circle_trajectory(t, radius=2.0, height=5.0, period=10.0):
        """
        Generate circular trajectory in x-y plane
        
        Args:
            t: time
            radius: circle radius
            height: z height
            period: time to complete one circle
            
        Returns:
            state: reference state at time t
        """
        omega = 2 * np.pi / period
        
        state = np.zeros(12)
        state[0] = radius * np.cos(omega * t)      # x
        state[1] = radius * np.sin(omega * t)      # y
        state[2] = height                           # z
        state[3] = -radius * omega * np.sin(omega * t)  # vx
        state[4] = radius * omega * np.cos(omega * t)   # vy
        state[5] = 0                                # vz
        # All angles and angular rates zero
        
        return state
    
    @staticmethod
    def figure_eight_trajectory(t, scale=2.0, height=5.0, period=20.0):
        """
        Generate figure-8 trajectory (lemniscate)
        
        Args:
            t: time
            scale: size scaling
            height: z height
            period: time to complete one loop
            
        Returns:
            state: reference state at time t
        """
        omega = 2 * np.pi / period
        
        state = np.zeros(12)
        
        # Lemniscate parametric equations
        state[0] = scale * np.sin(omega * t)                    # x
        state[1] = scale * np.sin(omega * t) * np.cos(omega * t) # y
        state[2] = height                                        # z
        
        # Velocities (derivatives)
        state[3] = scale * omega * np.cos(omega * t)
        state[4] = scale * omega * (np.cos(2 * omega * t))
        state[5] = 0
        
        return state
    
    @staticmethod
    def helix_trajectory(t, radius=2.0, height_rate=0.5, period=10.0):
        """
        Generate helical (spiral) trajectory
        
        Args:
            t: time
            radius: helix radius
            height_rate: vertical speed (m/s)
            period: time for one horizontal rotation
            
        Returns:
            state: reference state at time t
        """
        omega = 2 * np.pi / period
        
        state = np.zeros(12)
        state[0] = radius * np.cos(omega * t)           # x
        state[1] = radius * np.sin(omega * t)           # y
        state[2] = height_rate * t                      # z (climbing)
        state[3] = -radius * omega * np.sin(omega * t)  # vx
        state[4] = radius * omega * np.cos(omega * t)   # vy
        state[5] = height_rate                          # vz
        
        return state
    
    @staticmethod
    def waypoint_trajectory(t, waypoints, times):
        """
        Generate trajectory through waypoints with linear interpolation
        
        Args:
            t: current time
            waypoints: list of waypoint positions [[x,y,z], ...]
            times: times to reach each waypoint
            
        Returns:
            state: reference state at time t
        """
        state = np.zeros(12)
        
        # Find which segment we're in
        if t <= times[0]:
            state[0:3] = waypoints[0]
            return state
        
        if t >= times[-1]:
            state[0:3] = waypoints[-1]
            return state
        
        # Linear interpolation between waypoints
        for i in range(len(times) - 1):
            if times[i] <= t <= times[i+1]:
                alpha = (t - times[i]) / (times[i+1] - times[i])
                pos = (1 - alpha) * np.array(waypoints[i]) + alpha * np.array(waypoints[i+1])
                state[0:3] = pos
                
                # Compute velocity
                vel = (np.array(waypoints[i+1]) - np.array(waypoints[i])) / (times[i+1] - times[i])
                state[3:6] = vel
                break
        
        return state
    
    @staticmethod
    def hover_trajectory(position=[0, 0, 5]):
        """
        Simple hover at fixed position
        
        Args:
            position: [x, y, z] position to hover at
            
        Returns:
            state: reference state
        """
        state = np.zeros(12)
        state[0:3] = position
        return state
    
    @staticmethod
    def minimum_snap_trajectory(waypoints, times, t):
        """
        Generate minimum snap trajectory through waypoints
        (Simplified version using polynomial interpolation)
        
        Args:
            waypoints: list of waypoint positions
            times: times to reach waypoints
            t: current time
            
        Returns:
            state: reference state
        """
        # For simplicity, use polynomial interpolation
        # Full minimum snap would require QP optimization
        
        if t <= times[0]:
            state = np.zeros(12)
            state[0:3] = waypoints[0]
            return state
        
        if t >= times[-1]:
            state = np.zeros(12)
            state[0:3] = waypoints[-1]
            return state
        
        # Find segment
        for i in range(len(times) - 1):
            if times[i] <= t <= times[i+1]:
                # 5th order polynomial for smooth acceleration
                dt = times[i+1] - times[i]
                tau = (t - times[i]) / dt
                
                # Boundary conditions: position, velocity, acceleration at both ends
                # Simplified: just use smooth polynomial
                s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
                s_dot = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / dt
                
                p0 = np.array(waypoints[i])
                p1 = np.array(waypoints[i+1])
                
                state = np.zeros(12)
                state[0:3] = p0 + s * (p1 - p0)
                state[3:6] = s_dot * (p1 - p0)
                
                return state
        
        return np.zeros(12)
