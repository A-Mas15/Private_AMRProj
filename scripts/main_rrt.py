#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetModelState
from traj_generator import create_linear_trajectory
from map import Map
from RRT import RRTStar

class PathPublisher:
    """
    Creates and pubished the path using RRTStar
    """   
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
        start_x, start_y, start_theta = self.map.get_start()
        goal_x, goal_y, goal_theta = self.map.get_goal()

        if start_x is None:
            rospy.logerr("Start not found! Exiting.")
            return
        
        if goal_x is None:
            rospy.logerr("Goal not found! Exiting,")

        # Generates the trajectory
        rospy.loginfo("Running RRT* Path Planning...")
        rrt = RRTStar(self.map)
        for _ in range(RRTStar.MAX_ITER):
            rrt.update_tree()
        rrt.output_best_path()

        waypoints = [(x, y, theta) for segment in rrt.best_path for (x, y, theta) in segment]
        trajectory = Unicycle().plan_trajectory(waypoints[0], waypoints[-1])["points"]

        path_msg = Path()
        path_msg.header.frame_id = "map"
        for x, y, theta in trajectory:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            quaternion = euler_from_quaternion([0, 0, theta])
            pose.pose.orientation.w = quaternion[3]
            pose.pose.orientation.z = quaternion[2]
            path_msg.poses.append(pose)

        rospy.loginfo("Waiting for subscribers to /planned_path...")
        while self.pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.pub.publish(path_msg)
        rospy.loginfo("RRT* Path published successfully!")

