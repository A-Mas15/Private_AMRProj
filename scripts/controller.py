#!/usr/bin/env python3
import numpy as np
import math


class Controller:
    # Controller to track the trajectory
    def __init__(self, dt):
        # Control gains
        self.Kp = 15           # Position gain
        self.Kd = 10           # Velocity gain
        self.k_v = self.Kp
        # self.k_omega = 1.5      # Angular acceleration gain

        # Some bounds on acceleration
        self.max_dot_v = 0.5
        self.max_dot_omega = 1.0

        # Velocity threshold
        self.v_th = 0.05

        self.dt = dt

    def track_trajectory(self, x_d, dx_d, ddx_d, x_curr, dx_curr):
        """
        Computes control inputs (v, omega) to follow a trajectory.

        Args:
            x_d: desired trajectory (x(t), y(t)) in terms of flat output
            dx_d: desired velocity (dx(t), dy(t))
            ddx_d: desired acceleration (ddx(t), ddy(t))
            x_curr: current pose (x, y, theta)
            dx_curr: current velocity (v, omega)
            dt: time step, directly from the unicycle_kinematics

        Returns:
            v_curr: linear velocity command
            omega_curr: angular velocity command
            """
        
        p_error = np.array(x_d) - np.array(x_curr[:2])      # Position error
        v_error = np.array(dx_d) - np.array(dx_curr)        # Velocity error

        # Feedforward + PD controller
        a_tot =  np.array(ddx_d) + self.Kd*v_error + self.Kp*p_error 

        # Compute control inputs using Dynamic Feedback Linearization
        theta = x_curr[2]  # Current heading
        v_curr = dx_curr[0]  # Current linear velocity
        omega_curr = dx_curr[1]


        # If v_curr is too small, use a P controller
        if(abs(v_curr) < self.v_th):
            v_cmd = self.k_v * v_error[0]
            omega_cmd = 0
        else:
            print('ESEGUO IL FWD +PD')
            # Transformation matrix
            T = np.array([[math.cos(theta), -v_curr * math.sin(theta)],
                      [math.sin(theta),  v_curr * math.cos(theta)]])
            u = np.linalg.inv(T) @ a_tot  

            a_cmd = u[0] # Acceleration comand
            omega_cmd = u[1] # Angular velocity comand

            # # Apply acceleration limits
            # dot_v = max(min(a_cmd, self.max_dot_v), -self.max_dot_v)
            # dot_omega = max(min(omega_cmd, self.max_dot_omega), -self.max_dot_omega)
            # # Integrates velocities
            # v_curr += dot_v * dt
            # omega_curr += dot_omega * dt

            # Integrates the dynamic extended acceleration command
            v_cmd = v_curr + a_cmd * self.dt

            print(f"ACC: {a_cmd}, ANG VEL: {omega_curr}")
 

        return v_cmd, omega_cmd     # v_curr, omega_curr
