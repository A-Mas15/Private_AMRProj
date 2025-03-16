#!/usr/bin/env python3
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from RRT import RRTStar
from map import Map
import math, rospy

def compute_heading(pt1, pt2):
    # Compute the angle between two points (pt1 and pt2)
    return math.atan2(pt2[1] - pt1[1], pt2[0] - pt1[0])

def publish_path(waypoints):
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "map"  # Adjust frame as needed

    for i, point in enumerate(waypoints):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        # Fixed as 0 since we're on the ground
        pose.pose.position.z = 0

        # Compute the orientation
        if i < len(waypoints) - 1:
            heading = compute_heading(point, waypoints[i + 1])
        # For the last point it uses the previous heading or set to 0 if there is only one point in the path
        else:  
            if i > 0:
                prev_point = waypoints[i - 1]
                heading = compute_heading(prev_point, point)
            else:  
                heading = 0.0  

        # Convert heading to a quaternion 
        quaternion = quaternion_from_euler(0.0, 0.0, heading) # Roll, Pitch, Yaw
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        
        path_msg.poses.append(pose)

    # Publish the path message
    pub.publish(path_msg)

if __name__ == '__main__':
    rospy.init_node('rrt_path_publisher')
    pub = rospy.Publisher('/planned_path', Path, queue_size=10)
    # RRTStar output_best_path returns the best path, so a list of waypoints 
        # Create an instance of the map
    map = Map(msg)
    # Create an instance of the RTT* alg
    rrtS= RRTStar(map)
    waypoints = rrtS.output_best_path() 
    publish_path(waypoints)
