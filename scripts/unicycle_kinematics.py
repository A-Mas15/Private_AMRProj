#!/usr/bin/env python3
import numpy as np
import math

class Unicycle():
    """this class is used to plan a trajectory between two configurations of the unicycle 
        exploiting the differential flatness property"""
    
    def __init__(self):
        self.parameters = {'smoothness': 100,
                           'trajectory_time': 1,
                           'trajectory_sampling_step': 0.005}
        self.dt =self.parameters['trajectory_sampling_step']
    
    def plan_trajectory(self, q_in, q_fin, map_width = None, map_height = None):
        """Hyperparameters: -T: trajectory time (possible improvement we can implement minimum time tarjectory planning)
                            -k: smoothness of path
        """
        # scaling of the points(if need)
        x_in = q_in[0]      #/ map_width                   # scaled in [0,1]
        y_in = q_in[1]      #/ map_height                  # scaled in [0,1]
        theta_in = q_in[2]  #/ math.pi/2                   # scaled in [-1,1]

        x_fin = q_fin[0]    #/ map_width
        y_fin = q_fin[1]    #/ map_height
        theta_fin = q_fin[2] #/ math.pi/2

        trajectory = {'points' : [] ,'inputs' : [] }

        t=0
        T = self.parameters['trajectory_time']
        # Trajectory planning with cubic polinomials from q_in to q_fin
        while round(t,2) <= T:

            # timing law:
            s = t / T           # s = alpha * t

            # smoothness parameter:
            k = self.parameters['smoothness']       

            alpha_x = k * math.cos(theta_fin) - 3 * x_fin
            beta_x = k * math.cos(theta_in) + 3 * x_in

            alpha_y = k * math.sin(theta_fin) - 3 * y_fin
            beta_y = k * math.sin(theta_in) + 3 * y_in
            
            # Cubic polynomials
            x_t = s**3 * x_fin - (s-1)**3 * x_in + alpha_x * s**2*(s-1) + beta_x * s *(s-1)**2
            y_t = s**3 * y_fin - (s-1)**3 * y_in + alpha_y * s**2*(s-1) + beta_y * s *(s-1)**2

            dx = 3*s**2 * x_fin - 3*(s-1)**2 * x_in + alpha_x * s *(3*s - 2)+ beta_x *(3*s**2 - 4*s +1)
            dy = 3*s**2 * y_fin - 3*(s-1)**2 * y_in + alpha_y * s *(3*s - 2)+ beta_y *(3*s**2 - 4*s +1)

            dx_t = dx * 1/T
            dy_t = dy * 1/T

            ddx = 6*s * x_fin - 6*(s-1) * x_in + 2 * alpha_x *(3*s - 1)+ 2 * beta_x *(3*s - 2)
            ddy = 6*s * y_fin - 6*(s-1) * y_in + 2 * alpha_y *(3*s - 1)+ 2 * beta_y *(3*s - 2)

            ddx_t = ddx * 1/(T ** 2)
            ddy_t = ddy * 1/(T ** 2)

            # Recontstruction Formulas
            theta = math.atan2(dy_t,dx_t)
            v_t = math.sqrt(dx_t **2 + dy_t**2) 
            w_t = (ddy_t * dx_t - dy_t * ddx_t)/(dx_t**2 + dy_t**2) 
    
            t += self.dt
            trajectory['inputs'].append([v_t,w_t])
            trajectory['points'].append((x_t,y_t))

            #update cost
            #trajectory['cost'] = trajectory['cost'] + v_t * self.dt

        return trajectory
        # prova
        t=0
        i=0
        q_traj = []
        x, y, theta = q_in
        while t< T:
            v, omega = primitive['inputs'][i]
            # Euler integration
            x = x + v * self.dt * math.cos(theta)
            y = y + v * self.dt * math.sin(theta)
            theta = theta + omega * self.dt
    
            t += self.dt
            i+=1
            q_traj.append((x,y))

        return q_traj

    def get_path_length_and_smoothness(self, v1, v2):
        x_in = v1[0]
        y_in = v1[1] 
        theta_in = v1[2] 

        x_fin = v2[0]
        y_fin = v2[1] 
        theta_fin = v2[2]

        t=0
        T = self.parameters['trajectory_time']

        cost = 0
        smooth = 0
        theta_prev = theta_in
        # Trajectory planning with cubic polinomials from q_in to q_fin
        while round(t,2) <= T:

            # timing law:
            s = t / T           # s = alpha * t

            # smoothness parameter:
            k = self.parameters['smoothness']          

            alpha_x = k * math.cos(theta_fin) - 3 * x_fin
            beta_x = k * math.cos(theta_in) + 3 * x_in

            alpha_y = k * math.sin(theta_fin) - 3 * y_fin
            beta_y = k * math.sin(theta_in) + 3 * y_in
            
            # Cubic polynomials
            dx = 3*s**2 * x_fin - 3*(s-1)**2 * x_in + alpha_x * s *(3*s - 2)+ beta_x *(3*s**2 - 4*s +1)
            dy = 3*s**2 * y_fin - 3*(s-1)**2 * y_in + alpha_y * s *(3*s - 2)+ beta_y *(3*s**2 - 4*s +1)

            dx_t = dx * 1/T
            dy_t = dy * 1/T

            # Recontstruction Formulas
            theta = math.atan2(dy_t,dx_t)
            v_t = math.sqrt(dx_t **2 + dy_t**2) 
    
            t += self.dt

            #update cost
            cost = cost + v_t * self.dt
            smooth = smooth + abs(theta - theta_prev) 
            theta_prev = theta

        return cost, smooth


    def get_arc_length(self, q_traj):
        """return arclength following q_traj"""
        L=0
        q_prev = q_traj[0]
        for q in q_traj[1:]:
            L += math.sqrt((q_prev[0]-q[0]) ** 2 + (q_prev[1]-q[1]) ** 2)
            q_prev = q
        return L
    


if __name__ == '__main__':

    # for DEBUGGING
    robot = Unicycle()

    print(robot.primitives['direction'][1])

    import random, math
    # x = random.randint(0, 700)
    # y = random.randint(0, 500)
    # theta = random.uniform(-math.pi/2, math.pi/2)
    # print('x', x, 'y', y, 'theta', theta)

    # q = (x,y,theta)
    # q_finals = robot.get_primitives_final_points(q)
    # print('q_finals left', q_finals[0])
    # print('q_finals center', q_finals[1])
    # print('q_finals right', q_finals[2])

    # Q = robot.generate_primitives(q)
    #print('Q left', Q[0])
    # print('Q center', Q[1])
    # print('Q right', Q[2])

