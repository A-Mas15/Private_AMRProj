#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion
import math
from math import cos as c
from math import sin as s
import numpy as np
from map import Map
from unicycle_kinematics import Unicycle


class PathPublisher:
    """ Creates and pubished the path """   
    def __init__(self):
        # Initialize the node for path planner and publishes on it
        rospy.init_node('path_planner', anonymous=True)
        self.pub = rospy.Publisher('/planned_path', Path, queue_size=10)
        
        # Subscribes to the map
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.map = None
        rospy.loginfo("Waiting for the map to be published...")
        
        # Waits 10s for the map
        try:
            msg = rospy.wait_for_message("/map", OccupancyGrid, timeout=10)
            self.map_callback(msg)
        except rospy.ROSException:
            rospy.logerr("Timed out waiting for the map! Make sure map_server is running.")
            return

        # Some buffer time, to avoid starting planning process before the map is fully initialized
        rospy.sleep(1)
        self.publish_path()


    # Stores the map data
    def map_callback(self, msg):
        if self.map is None: # Only initialize the map once!
            self.map = Map(msg)
            rospy.loginfo("Map received and initialized.")

    def publish_path(self):
        if self.map is None:
            rospy.logerr("Map not initialized, cannot generate path.")
            return

        # Get start and goal position from the map
        q_in = self.map.get_start()
        q_f = self.map.get_goal()

        if q_in is None:
            rospy.logerr("Start not found! Exiting.")
            return
        
        if q_f is None:
            rospy.logerr("Goal not found! Exiting,")

        # Generates the trajectory
        trajectory = Unicycle().plan_trajectory(q_in, q_f)["points"]

        path_msg = Path()
        path_msg.header.frame_id = "map"

        # Formats the path for a ROS message
        for x, y in trajectory:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            theta = math.atan2(y, x)
            quaternion = euler_from_quaternion([0, 0, theta])
            pose.pose.orientation.w = quaternion[3]
            pose.pose.orientation.z = quaternion[2]
            path_msg.poses.append(pose)

        rospy.loginfo("Waiting for subscribers to /planned_path...")
        while self.pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.pub.publish(path_msg)
        rospy.loginfo("Path published successfully!")


class Controller:
    """Class that subscribes to the planned path and odometry, then computes velocity commands."""

    def __init__(self):
        # Initialize the node and substives to planned_path and odometry
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/planned_path', Path, self.path_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.trajectory = []
        self.current_index = 0
        self.unicycle = Unicycle()
        self.rate = rospy.Rate(10)

        # Robot state
        self.rob_x = 0
        self.rob_y = 0
        self.rob_theta = 0
        self.v_curr = 0.0
        self.omega_curr = 0.0

        # Control gains
        self.Kp, self.Kd = 1.5, 1.0
        self.k_v, self.k_omega = 1.2, 1.5
        self.dt = 0.1
        self.max_dot_v, self.max_dot_omega = 0.5, 1.0

    def path_callback(self, msg):
        # Receives the planned path and stores it as a trajectory
        waypoints = [(pose.pose.position.x, pose.pose.position.y,
                      euler_from_quaternion([pose.pose.orientation.x, 
                                             pose.pose.orientation.y, 
                                             pose.pose.orientation.z, 
                                             pose.pose.orientation.w])[2]) for pose in msg.poses]
        # Iterate through the waypoints in pairs to generate trajectory segments
        for i in range(len(waypoints) - 1):
            segment = Unicycle().plan_trajectory(waypoints[i], waypoints[i+1])["points"]

            # Append the segment to the full trajectory
            self.trajectory.extend(segment["points"])
            # Save also velocity commands
            self.inputs.extend(segment["inputs"])

        # Reset index for trajectory tracking
        self.current_index = 0
        rospy.loginfo("Received trajectory with %d points.", len(self.trajectory))

    def odom_callback(self, msg):
        # Updates the robot's current state (pose and velocities) from odometry
        self.rob_x = msg.pose.pose.position.x
        self.rob_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, self.rob_theta) = euler_from_quaternion([orientation_q.x, orientation_q.y,
                                                        orientation_q.z, orientation_q.w])
        self.v_curr = msg.twist.twist.linear.x
        self.omega_curr = msg.twist.twist.angular.z

    def run(self):
        # Control loop

        # Since will be used here and only here, I'll define it as a local variable just in this method
        v_th = 0.05
        while not rospy.is_shutdown():
            if not self.trajectory:
                rospy.logwarn("No trajectory available. Waiting for path...")
                continue

            if self.current_index < len(self.trajectory):
                # Where we are on the trajectory. x_d on the notes
                p_des = np.array(self.trajectory[self.current_index])
                
                # Given from the Unicycle class
                v_des, omega_des = self.inputs[self.current_index]
                desired_theta = math.atan2(p_des[1] - self.rob_y, p_des[0] - self.rob_x)

                p = np.array([self.rob_x, self.rob_y])

                # Errors
                p_error = p_des - p   # Position error. x_d - x in the notes
                heading_error = self.normalize_angle(desired_theta - self.rob_theta)
                v_error = v_des - self.v_curr # Velocity error. \dot{x}_d - \dot{x} in the notes
                omega_error = omega_des - self.omega_curr # Angular velocity error

                # PD Controller
                # Computes desired a. Missing ddot{x}. Need a way to get it.
                a_d = self.Kp * p_error + self.Kd * v_error

                # If v_curr is too small, use a P controller
                if(abs(self.v_curr) < v_th):
                    self.v_curr = self.k_v * v_error
                    self.omega_curr = 0
                    # If we want to correct also rotation as the robot moves 
                    # self.omega_curr = self.k_omega * heading_error
                else:
                    # Matrix T
                    T = np.array([c(self.rob_theta), - self.v_curr*s(c(self.rob_theta))],
                                 [s(self.rob_theta), self.v_curr*c(self.rob_theta)])
                    u = np.dot(np.linalg.inv(T), a_d)
                    a_cmd = u[0] # Acceleration comand
                    omega_cmd = u[1] # Angular velocity comanf
                    # Apply acceleration limits
                    dot_v = max(min(a_cmd, self.max_dot_v), -self.max_dot_v)
                    dot_omega = max(min(omega_cmd, self.max_dot_omega), -self.max_dot_omega)
                    # Integrates velocities
                    self.v_curr += dot_v * self.dt
                    self.omega_curr += dot_omega * self.dt

                # Publish command
                twist = Twist()
                twist.linear.x = self.v_curr
                twist.angular.z = self.omega_curr
                self.cmd_pub.publish(twist)

                if np.linalg.norm(p_error) < 0.1:
                    self.current_index += 1
                    rospy.loginfo("Reached trajectory point %d/%d", self.current_index, len(self.trajectory))

            self.rate.sleep()

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

if __name__ == "__main__":
    path_publisher = PathPublisher()
    controller = Controller()
    controller.run()
