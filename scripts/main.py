#!/usr/bin/env python3
import rospy, time
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from controller import Controller
from map import Map
from RRT import RRTStar


def main():
    # Some global variables
    global cmd_pub, map, current_pose, current_velocity

    # Step 1: initialization
    rospy.init_node('rrt_planner', anonymous=True)

    # Subscribe to topic /odom, to get information on the robot as it moves
    rospy.Subscriber('/odom', Odometry, odom_callback)
    # Publish velocity comands on the topic /cmd_vel
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.loginfo("Waiting for the map to be published...")
    # PROVA
    rate = rospy.Rate(10)
    map = None
    executed_path = {'nodes': [], 'edges': []}

    # Step 2: extract map data
    try:
        msg = rospy.wait_for_message("/map", OccupancyGrid, timeout=10)
        map = Map(msg)
    except rospy.ROSException:
        rospy.logerr("Timed out waiting for the map! Make sure map_server is running.")
        return
    # Some buffer time, to avoid starting planning process before the map is fully initialized
    rospy.sleep(1)
    map_width, map_height = map.width, map.height

     # Get start and goal position from the map
    start = map.start
    goal = map.goal
    current_pose = start
    executed_path['nodes'].append(current_pose)
    goal_x, goal_y, goal_theta = goal 

    if current_pose is None:
        rospy.logerr("Start not found! Exiting.")
        return
        
    if goal_x is None:
        rospy.logerr("Goal not found! Exiting,")

    # Step 3: path planning via RRT*
    rrt = RRTStar(map)
    join_start = False
    for iter in range(rrt.MAX_ITER):
        if iter >= rrt.MAX_ITER -1: # if it is the last iteration, try to join the goal
            join_start =True

        string = rrt.update_tree(join_start= join_start)
        if string == 'No path exist':
            rospy.logerr(f'No possible paths have been found in {rrt.MAX_ITER} iterations') 
            return
        
    rrt.output_best_path()
    print('path generated:', rrt.best_path)
    rospy.loginfo("RRT path planning complete. Starting trajectory tracking")

    # Step 4: trajectory tracking via PD+DFL controller
    x_prev = current_pose
    dx_prev = np.array([0.0, 0.0])
    dt = rrt.unicycle.dt
    controller = Controller(dt)
    #while current_pose != rrt.goal: due to approximation error, this condition may not be met. Better use this one
    while np.linalg.norm(np.array(current_pose[:2]) - np.array(rrt.goal[:2])) > 0.1:
        # map.update_slam()                         # TO BE DONE
        # new_map = map.get_map()                   # not necessary since when passing MAP to the class RRT it refers to the reference of the object (it updates when the original object updates)
        #rrt.update_tree_structure()

        next_edge, next_node = rrt.compute_next_path_node(current_pose, executed_path['nodes'])
        
        #if find_obstacle(next_edge):
        #    next_edge, next_node = rrt.replan_path(current_pose, next_node)
        
        if executed_path['nodes'][-1] != next_node:      #if next_node not in executed_path['nodes']:
            executed_path['nodes'].append(next_node)
            executed_path['edges'].append(next_edge)
            trajectory_to_track = next_edge
            #trajectory_to_track = rrt.best_path
            i = 0

        if i > len(trajectory_to_track) - 1:
            break

        x_d = trajectory_to_track[i][:2]
        dx_d = (np.array(x_d[:2]) - np.array(x_prev[:2])) / dt
        ddx_d = (dx_d - dx_prev) / dt
        x_current = get_current_pose()
        dx_current = get_current_velocity()
        v, omega = controller.track_trajectory(x_d, dx_d, ddx_d, x_current, dx_current)
        dx_prev = (np.array(x_current[:2]) - np.array(x_prev[:2]))/dt
        x_prev = np.array(x_current)

        i+=1

        # Publish command
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = omega
        cmd_pub.publish(twist)

        rospy.loginfo(f"Following trajectory: {x_d}")

        rate.sleep()  # Small delay to ensure smooth execution

        # for edge in trajectory_to_track:  # Loop through the entire planned path
        #     for x_d in edge:  #  Loop through each point in the edge
        #         dx_d = (np.array(x_d[:2]) - np.array(x_prev[:2])) / dt
        #         ddx_d = (dx_d - dx_prev) / dt
        #         x_current = get_current_pose()
        #         dx_current = get_current_velocity()
        #         v, omega = controller.track_trajectory(x_d, dx_d, ddx_d, x_current, dx_current)
        #         dx_prev = (x_current - x_prev)/dt
        #         x_prev = x_current
        
        #         # Publish command
        #         twist = Twist()
        #         twist.linear.x = v
        #         twist.angular.z = omega
        #         cmd_pub.publish(twist)
        
        #         rospy.loginfo(f"Following trajectory: {x_d}")
        
        #         time.sleep(0.1)  # Small delay to ensure smooth execution

    # Publish command
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    cmd_pub.publish(twist)

    rospy.loginfo(f"FINISH TRAJECTORY")

    time.sleep(0.1)  


# To process robot odometry data (pose, linear and angular velocity)
def odom_callback(msg):
    global current_pose, current_velocity
    position = msg.pose.pose.position # Position of the robot
    #orientation = msg.pose.pose.orientation # Pose of the robot in quaternion
    (_, _, theta) = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                             msg.pose.pose.orientation.y, 
                                             msg.pose.pose.orientation.z, 
                                             msg.pose.pose.orientation.w]) # We are only interested in the yaw angle, so on theta
    current_pose = position.x, position.y, theta
    current_velocity = [msg.twist.twist.linear.x, msg.twist.twist.angular.z]

def get_current_pose():
    # Returns the current pose od the robot
    if current_pose:
        return current_pose 
    else:
        map.start()

def get_current_velocity():
    # Returns the current velocity of the robot
    if current_velocity:
        return current_velocity 
    else:
        return [0.0, 0.0]

# Used for dynamic case
def find_obstacle(next_edge):
    # Cheks for the presence of an obstacle
    #Returns:
    #    True  -> If there is an obstacle
    #    False -> If there is NO obstacle
    if map is None:
        return False  # Assume no obstacles if the map is not available
    return not map.collision_free(next_edge)

if __name__ == "__main__":
    main()