#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path, OccupancyGrid
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from RRT import RRTStar


class Map:
    def __init__(self, occupancy_grid):
        # rospy.Subscriber("/odom", Odometry, self.odometry_callback)

        # Dimension of each cell in meters
        self.resolution = occupancy_grid.info.resolution 

        # Number of cells along the x axis
        self.width = occupancy_grid.info.width 
        # Number of cells along the y axis
        self.height = occupancy_grid.info.height
        # Debugging print
        print(f"Map size: {self.width} x {self.height}")
        # Origin of the map
        self.origin = occupancy_grid.info.origin          


        # Convert the flat occupancy data array into a 2D grid for easier indexing.
        self.grid = [occupancy_grid.data[i * self.width:(i + 1) * self.width]
                     for i in range(self.height)]
        
        # Define start and goal configurations (x, y, theta)
        self.goal_pose = self.get_goal_pose()
        
        # Directly from gazebo
        self.start_pose = self.get_start_pose()


        # For odometry
        #self.start_pose = None

        #if self.start_pose and self.goal_pose:
        #    rospy.loginfo("Start position: (%.2f, %.2f)", self.start_pose[0], self.start_pose[1])
        #    rospy.loginfo("Goal position: (%.2f, %.2f)", self.goal_pose[0], self.goal_pose[1])
  


    # Check if the trajectory is collision free
    def collision_free(self, trajectory):
        for point in trajectory:
            x, y = point[0], point[1]
            # Convert world coordinates to grid indices.
            grid_x = int((x - self.origin.position.x) / self.resolution)
            grid_y = int((y - self.origin.position.y) / self.resolution)
            

            # Check if start is obstacle?
            if(x, y) == (self.grid[0][0]):
                print("Color of the START pixel:", self.grid[0][0   ])
                continue

            # The robot can pass through the goal
            if(x, y) == (self.goal_pose[0], self.goal_pose[1]):
                print("Color of the GOAL pixel:", self.grid[grid_y][grid_x])
                continue

            # Check if the indices are within bounds.
            if grid_x < 0 or grid_x >= self.width or grid_y < 0 or grid_y >= self.height:
                rospy.logwarn(f"Point ({x:.2f}, {y:.2f}) is out of bounds!")
                return False  
            
            # If the cell value is 80 or more, we treat it as occupied. This value is defined in the definition of the world
            if self.grid[grid_y][grid_x] >= 80:
                rospy.logwarn(f"Point ({x:.2f}, {y:.2f}) is in an obstacle!")
                # Debugging print
                print("Color of the pixel:", self.grid[grid_y][grid_x])
                return False

        return True
    
    def get_goal_pose(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            response = get_model_state("goal_marker", "") 
            if response.success:
                print("Gaol position found at: (%.2f, %.2f)", response.pose.position.x, response.pose.position.y)
                rospy.loginfo("Gaol position found at: (%.2f, %.2f)", response.pose.position.x, response.pose.position.y)
                return (response.pose.position.x, response.pose.position.y, 0.0)
            else:
                rospy.logerr("Goal position not found")
                return None
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return None
        
    def get_start_pose(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            response = get_model_state("turtlebot3_waffle", "")
            if response.success:
                x, y = response.pose.position.x, response.pose.position.y
                orientation_q = response.pose.orientation
                (_, _, theta) = euler_from_quaternion([orientation_q.x, orientation_q.y,
                                                        orientation_q.z,
                                                        orientation_q.w])
                
                # Check if the start position is in a collision using `collision_free`
                if not self.collision_free([(x, y)]):  # Pass as a single-point trajectory
                    rospy.logwarn(f"Start position ({x:.2f}, {y:.2f}) is inside an obstacle! Finding nearest free space...")
                    x, y = self.find_nearest_free_space(x, y)
                print(f"Start position set at: x = {x}, y =  {y}, theta = {theta}")
                print(f"quaternion = {orientation_q}")
                eulers = euler_from_quaternion([orientation_q.x, orientation_q.y,
                                                        orientation_q.z,
                                                        orientation_q.w])
                print(f"Roll, pitch, yaw = {eulers}")
                rospy.loginfo(f"Start position set at: ({x:.2f}, {y:.2f}, {theta: 2f})")
                return (x, y, theta)
            else:
                rospy.logerr("Starting position not found")
                return None
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return None

#
#    def odometry_callback(self, msg):
#        rob_x = msg.pose.pose.position.x
#        rob_y = msg.pose.pose.position.y
#        orientation_q = msg.pose.pose.orientation
#        (_, _, rob_theta) = euler_from_quaternion([orientation_q.x, orientation_q.y,
#                                                   orientation_q.z, orientation_q.w])
#        
#        if not self.collision_free([(rob_x, rob_y)]):
#            rospy.logwarn(f"Start position ({rob_x:.2f}, {rob_y:.2f}) is inside an obstacle! Finding nearest free space...")
#            rob_x, rob_y = self.find_nearest_free_space(rob_x, rob_y)
#        
#        self.start_pose = (rob_x, rob_y, rob_theta)
#        rospy.loginfo(f"Start position set at: ({rob_x:.2f}, {rob_y:.2f}, {rob_theta:.2f})")
#
    def find_nearest_free_space(self, x, y):
        step_size = self.resolution  # Move in grid-sized steps
        max_search_radius = 10  # Limit search radius

        for radius in range(1, max_search_radius):
            for dx in [-radius, 0, radius]:
                for dy in [-radius, 0, radius]:
                    new_x, new_y = x + dx * step_size, y + dy * step_size
                    if self.collision_free([(new_x, new_y)]):  # Check if free
                        rospy.loginfo(f"Found free start position at: ({new_x:.2f}, {new_y:.2f})")
                        return new_x, new_y

        rospy.logwarn("No nearby free space found! Keeping original start position.")
        return x, y  # If no free space found, return original



    # Getters so that class RRT can access start and goal pose
    def get_start(self):
        return self.start_pose
    
    def get_goal(self):
        return self.goal_pose