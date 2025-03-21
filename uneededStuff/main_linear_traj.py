#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from traj_generator import create_linear_trajectory
from map import Map

"""
A simple script to check if the controller works with a simple linear trajectory.
"""

class PathPublisher:
    def __init__(self):
        rospy.init_node('path_publisher')
        self.pub = rospy.Publisher('/planned_path', Path, queue_size=10)

        # Subscribe to /map topic
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.map = None

        rospy.loginfo("Waiting for the map to be published...")
        try:
            msg = rospy.wait_for_message("/map", OccupancyGrid, timeout=10)  # Timeout in 10 sec
            self.map_callback(msg)
        except rospy.ROSException:
            rospy.logerr("Timed out waiting for the map! Make sure map_server is running.")
            return

        rospy.sleep(1)  # Allow some buffer time
        self.publish_path()

    def map_callback(self, msg):
        if self.map is None:  # Only initialize once
            self.map = Map(msg)
            rospy.loginfo("Map received and initialized.")

    def publish_path(self):
        if self.map is None:
            rospy.logerr("Map not initialized, cannot generate path.")
            return

        # Get start and goal positions
        start_x, start_y, start_theta = self.map.get_start()
        goal_x, goal_y, goal_theta = self.map.get_goal()

        if start_x is None or goal_x is None:
            rospy.logerr("Start or Goal not found! Exiting.")
            return

        rospy.loginfo(f"Generating trajectory from ({start_x}, {start_y}) to ({goal_x}, {goal_y}) using traj_generator")

        # Generate trajectory using traj_generator
        trajectory = create_linear_trajectory(start_x, start_y, goal_x, goal_y)

        # Ensure there is at least one subscriber before publishing
        rospy.loginfo("Waiting for subscribers to /planned_path...")
        while self.pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.pub.publish(trajectory)
        rospy.loginfo("Trajectory published successfully!")

        rospy.spin()

if __name__ == "__main__":
    PathPublisher()
