#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

"""
A simple script to check if the controller works with a simple linear trajectory
"""

"""
Get the position and orientation of the robot directly from Gazebo
"""
def get_robot_pose():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        response = get_model_state("turtlebot3_waffle", "")  

        if response.success:
            x = response.pose.position.x
            y = response.pose.position.y
            orientation_q = response.pose.orientation

            # Theta is the theta angle
            (_, _, theta) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

            rospy.loginfo(f"Robot start position: x={x:.2f}, y={y:.2f}, theta={theta:.2f} rad")
            return x, y, theta
        else:
            rospy.logerr("Failed to get robot state from Gazebo")
            return None, None, None
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None, None, None

"""
Get the target position, defined in Gazebo using a goal_marker
"""
def get_goal_pose():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        response = get_model_state("goal_marker", "") 
        if response.success:
            print("Gaol position found at: (%.2f, %.2f)", response.pose.position.x, response.pose.position.y)
            rospy.loginfo("Goal position found at: (%.2f, %.2f)", response.pose.position.x, response.pose.position.y)
            return (response.pose.position.x, response.pose.position.y, 0.0)
        else:
            rospy.logerr("Goal position not found")
            return None
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None

"""
Generates the trajectory. The goal_marker has been set in Gazebo as static, so its position is known (0, ~3)
"""
def create_straight_path(start_x, start_y, start_theta, goal_x, goal_y):
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "map"

    # Define waypoints in a straight line toward the goal (0,4)
    waypoints = [(start_x, start_y), (0,1), (0,2), (0,3), (goal_x, goal_y)]

    for point in waypoints:
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = 0 

        # Use the robot's initial theta angle
        quaternion = quaternion_from_euler(0, 0, start_theta)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        path_msg.poses.append(pose)

    return path_msg

def main():
    rospy.init_node('manual_path_publisher')
    pub = rospy.Publisher('/planned_path', Path, queue_size=10)

    rospy.sleep(1)  # Wait for the publisher to initialize

    # Get the robot's current position and orientation
    start_x, start_y, start_theta = get_robot_pose()
    goal_x, goal_y, _ = get_goal_pose()

    if start_x is None:
        rospy.logerr("Could not retrieve robot pose. Exiting.")
        return

    # Create and publish the path
    path_msg = create_straight_path(start_x, start_y, start_theta, goal_x, goal_y)
    pub.publish(path_msg)

    rospy.loginfo("Path published successfully!")
    rospy.spin()  # Keeps the node alive

if __name__ == "__main__":
    main()
