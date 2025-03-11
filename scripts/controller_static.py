#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

class Controller:

    def __init__(self):
        # MODIFIED 
        rospy.init_node('controller', anonymous=True)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # The topic planned_path is defined in waypoints.py
        rospy.Subscriber('/planned_path', Path, self.path_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.waypoints = []
        self.current_index = 0
        self.rate = rospy.Rate(10)
        # Initialize the robot pose as 0.0
        self.rob_x = 0.0
        self.rob_y = 0.0
        self.rob_theta = 0.0

    def path_callback(self, msg):
        # msg.poses is the list of waypoints
        self.waypoints = msg.poses
        self.current_index = 0

    def odom_callback(self, msg):
        self.rob_x = msg.pose.pose.position.x
        self.rob_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, self.rob_theta) = euler_from_quaternion([orientation_q.x, orientation_q.y,
                                                orientation_q.z,
                                                orientation_q.w])
        rospy.loginfo("Current pose: (%.2f, %.2f, %.2f)", self.rob_x, self.rob_y, self.rob_theta)

    def run(self):
        while not rospy.is_shutdown():
            if self.waypoints and self.current_index < len(self.waypoints):
                target_pose = self.waypoints[self.current_index].pose
    
                # Compute desired heading and distance
                des_x = target_pose.position.x - self.rob_x
                des_y = target_pose.position.y - self.rob_y
                des_theta = math.atan2(des_y, des_x)
                angle_error = self.normalize_angle(des_theta - self.rob_theta)
                distance_error = math.sqrt(des_x**2 + des_y**2)
    
                twist = Twist()
    
                # Rotation first
                if abs(angle_error) > 0.1:  # If misaligned, prioritize turning
                    twist.angular.z = 1.0 * angle_error
                    #twist.linear.x = 0.0  # Don't move forward while turning
                    twist.linear.x = 0.2 # Moves slowly forward
                    
                else:
                    twist.linear.x = 0.5 * distance_error
                    twist.angular.z = 0.0  # Stop turning
    
                self.cmd_pub.publish(twist)
    
                # Check if the waypoint is reached
                if distance_error < 0.1:
                    self.current_index += 1
                    rospy.loginfo(f"Reached waypoint {self.current_index}/{len(self.waypoints)}")
    
            else:
                # Stop when done
                self.cmd_pub.publish(Twist())
                rospy.loginfo("Navigation complete.")
            
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
