#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from RRT import RRTStar
from map import Map
from waypoints import publish_path 
from nav_msgs.msg import Path


def run_rrt(gazebo_map):
    rospy.loginfo("Running RRT* Path Planning: ")
    rrt = RRTStar(gazebo_map)

    for iter in range(rrt.MAX_ITER):
        if iter >= rrt.MAX_ITER - 1:
            join_goal = True
        else:
            join_goal = False

        status = rrt.update_tree(join_goal=join_goal)
        if status == 'No path exist':
            rospy.logerr(f'No paths found in {rrt.MAX_ITER} iterations.')
            return

    # Compute and publish best path
    best_path = rrt.output_best_path()  # Get best path waypoints
    rrt.aco_optimization()
    rospy.loginfo("Best path generated and optimized.")

    # Publish the path for the controller
    if best_path:
        publish_path(best_path)  # Publish the waypoints to /planned_path
        rospy.loginfo("Path has been published for execution.")

# Get the map
def map_callback(msg):
    rospy.loginfo("Occupancy grid received. Initializing RRT*: ")
    gazebo_map = Map(msg)

    # Wait until start_pose is set before running RRT*
    while (gazebo_map.get_start() is None or gazebo_map.get_goal() is None):
        rospy.loginfo("Waiting for start pose from odometry...")
        rospy.sleep(0.5)  # Allow time for odometry to be received

    run_rrt(gazebo_map)


def main():
    rospy.init_node('rrt_path_planner')

    # Ensure that we have a ROS publisher for the path
    global pub
    pub = rospy.Publisher('/planned_path', Path, queue_size=10)

    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
