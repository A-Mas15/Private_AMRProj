#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math


"""
    Generates a linear trajectory from (start_x, start_y) to (end_x, end_y).
    """
def create_linear_trajectory(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, num_points=50):
    trajectory = Path()
    trajectory.header.frame_id = "map"

    for i in range(num_points):
        alpha = i / (num_points - 1)  # Interpolation factor (0 to 1)
        x = start_x + alpha * (goal_x - start_x)
        y = start_y + alpha * (goal_y - start_y)
        theta = start_theta + alpha * (goal_theta - start_theta)
        

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y


        # Compute orientation (facing along the x-axis)
        #theta = math.atan2(goal_y - start_y, goal_x - start_x)
        pose.pose.orientation.w = math.cos(theta / 2)
        pose.pose.orientation.z = math.sin(theta / 2)

        trajectory.poses.append(pose)

    return trajectory
