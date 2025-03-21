#!/usr/bin/env python3
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math, rospy
import numpy as np
from unicycle_kinematics import Unicycle


class Controller:
    # Controller to track the trajectory
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/planned_path', Path, self.path_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Define the model and initialize the trajectory planner
        self.unicycle = Unicycle()
        self.trajectory = []
        self.current_index = 0
        self.rate = rospy.Rate(10)

        # Robot state, initially still
        self.v_curr = 0.0
        self.omega_curr = 0.0

        # Control gains
        self.Kp = 1.5           # Position gain
        self.Kd = 1.0           # Velocity gain
        self.k_v = 1.2          # Linear acceleration gain
        self.k_omega = 1.5      # Angular acceleration gain
        self.dt = 0.1           # Control update rate
        # Some bounds on acceleration
        self.max_dot_v = 0.5
        self.max_dot_omega = 1.0


    # Receives waypoints and computes a continuous trajectory
    def path_callback(self, msg):
        waypoints = [(pose.pose.position.x, pose.pose.position.y,
                    euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])[2]   ) for pose in msg.poses]
        self.trajectory = []
        #for i in range(len(waypoints)-1):
        #    traj = self.unicycle.plan_trajectory(waypoints[i], waypoints[i+1])["points"]
        #    self.trajectory.extend(traj)
        #    #self.trajectory.extend(traj['points'])  # Store trajectory points
        self.trajectory = self.unicycle.plan_trajectory(waypoints[0], waypoints[-1])["points"]
        print(f"Traiettoria: {self.trajectory}")
        self.current_index = 0


    # Get pose and velocities of the robot
    def odom_callback(self, msg):
        self.rob_x = msg.pose.pose.position.x
        self.rob_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, self.rob_theta) = euler_from_quaternion([orientation_q.x, orientation_q.y,
                                                orientation_q.z,
                                                orientation_q.w])
        self.v_curr = msg.twist.twist.linear.x
        self.omega_curr = msg.twist.twist.angular.z

        #rospy.loginfo("Current pose: (%.2f, %.2f, %.2f)", self.rob_x, self.rob_y, self.rob_theta)
        #rospy.loginfo("Current velocities: (%.2f, %.2f)", self.v_curr, self.omega_curr)

    def run(self):
        while not rospy.is_shutdown():
            if not self.trajectory:
                #rospy.logwarn("No trajectory available. Waiting for path")
                continue

            if self.trajectory and self.current_index < len(self.trajectory):
                # Compute desired trajectory
                p_des = np.array(self.trajectory[self.current_index])
                # Some placeholders
                a_des = np.array([0,0])

                # Actual pose
                p = np.array([self.rob_x, self.rob_y])
                # Position error
                e = p_des -p

                # PD controller
                a_fd = self.Kp * e - self.Kd * np.array([self.v_curr * math.cos(self.rob_theta), self.v_curr * math.sin(self.rob_theta)])# Feedback acceleration
                a_tot = a_des + a_fd

                # Desired velocities from accelerations using inverse transformation
                v_des = np.linalg.norm(a_tot)  
                # Compute heading error
                desired_theta = math.atan2(p_des[1] - self.rob_y, p_des[0] - self.rob_x)
                heading_error = self.normalize_angle(desired_theta - self.rob_theta)

                # If the heading error is large, prioritize rotation before moving
                if abs(heading_error) > 0.1:  # Rotate in place
                    # self.v_curr = 0.1  # Prioratize rotation
                    omega_des = 2.0 * heading_error  # Rotate towards the correct direction
                else:
                    omega_des = 2.0 * self.normalize_angle(math.atan2(a_tot[1], a_tot[0]) - self.rob_theta)  # Continue tracking
                
                
                
                # Compute accelerations
                dot_v = self.k_v * (v_des - self.v_curr)
                dot_omega = self.k_omega * (omega_des - self.omega_curr)

                # Limit accelerations
                dot_v = max(min(dot_v, self.max_dot_v), -self.max_dot_v)
                dot_omega = max(min(dot_omega, self.max_dot_omega), -self.max_dot_omega)

                # Compute new velocities by integrating the accelerations
                self.v_curr += dot_v * self.dt
                self.omega_curr += dot_omega * self.dt

                # Publish command
                twist = Twist()
                twist.linear.x = self.v_curr
                twist.angular.z = self.omega_curr
                self.cmd_pub.publish(twist)
                
                # Check if goal is reached
                if self.current_index >= len(self.trajectory) - 1 and np.linalg.norm(e) < 0.1 and abs(heading_error) < 0.1:
                    rospy.loginfo("Goal reached. Stopping the robot.")
                    self.cmd_pub.publish(Twist())
                    return
                
                # Check if the waypoint is reached
                if np.linalg.norm(e) < 0.1:
                    self.current_index += 1
                    rospy.loginfo(f"Reached trajectory point {self.current_index}/{len(self.trajectory)}")
                   
            self.rate.sleep()


    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle



if __name__ == '__main__':
    controller = Controller()
    controller.run()
