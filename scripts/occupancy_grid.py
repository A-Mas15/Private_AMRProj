#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from RRT import RRTStar 
from map import Map
from waypoints import publish_path

def occupancy_grid(msg):
    rospy.loginfo("Received map data.")
    # Create an instance of the map
    map = Map(msg)

    # Create an instance of the RTT* alg
    rrtS= RRTStar(map)

    # Run the planning algorithm
    for i in range(rrtS.MAX_ITER):
        tree = rrtS.update_tree()
        if tree == 'No path exist':
            rospy.logerr("No valid path exists")
            return
    # Get the best planner
    best_path = rrtS.output_best_path()
    rospy.loginfo("Best path is: %s", best_path)

    if best_path is not None:
        publish_path([(p[0][0], p[0][1]) for p in best_path])
    else:
        rospy.logerr("No path was found.")


def listener():
    rospy.init_node('map_listener', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, occupancy_grid)
    rospy.spin()

if __name__ == '__main__':
    listener()
