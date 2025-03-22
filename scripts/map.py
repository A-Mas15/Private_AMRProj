#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path, OccupancyGrid
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion
import numpy as np
import math

"""
The class Map takes an occupancy grid and extracts information on the map (resolution, width and height).

"""

class Map:
    def __init__(self, msg):
        self.resolution = msg.info.resolution    # Dimension of each cell in meters
        self.grid_width = msg.info.width              # Number of cells along the x axis
        self.grid_height = msg.info.height            # Number of cells along the y axis
        self.origin = msg.info.origin            # Origin of the map
        self.map = msg

        # Width and height in meter
        self.width  = self.grid_width  * self.resolution
        self.height = self.grid_height  * self.resolution

        # Convert the flat occupancy data array into a 2D grid for easier indexing.
        self.grid = [msg.data[i * self.grid_width:(i + 1) * self.grid_width]
                     for i in range(self.grid_height)]
        self.grid = np.array(self.grid)

        self.goal = self.get_goal_pose()
        self.start = self.get_start_pose()

        rospy.loginfo(f"Map: {self.grid_width}x{self.grid_height} celle, risoluzione = {self.resolution} m/cella")
        rospy.loginfo(f"Dimensione fisica della mappa: {self.width:.2f}m x {self.height:.2f}m")
        rospy.loginfo(f"Origine mappa (in metri): x = {self.origin.position.x}, y = {self.origin.position.y}")



    def collision_free(self, q_traj, neighborhood_size=3):
        """Check if a trajectory is collision-free considering a neighborhood.
        
        Args:
        q_traj: List of (x, y, theta) positions forming a trajectory.
        It expects coordinates in cells.
        
        Returns:
        True  -> If the trajectory is obstacle-free.
        False -> If any point in the trajectory is in collision."""
        for point in q_traj:
            if self.check_collision(point, neighborhood_size):
                return False  # Collision detected
        return True  # Safe trajectory

    def check_collision(self, config, neighborhood_size=1):
        """Check if a given configuration collides with an obstacle in the occupancy grid, 
           considering a neighborhood of `neighborhood_size` cells around the point.

        Args:
            config (tuple): The (x, y, theta) world coordinates to check.
            neighborhood_size (int): The half-width of the square neighborhood.
                                     A value of 3 means checking a 7x7 area (3+1+3).
        Returns:
            bool: True if there is a collision in the neighborhood, False otherwise.
        """
        x_world, y_world, _ = config  # Extract position from the map (meters)
        # Convert world coordinates to grid indices
        x, y = self.world_to_map(x_world, y_world)
        # Check if out of bounds
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            rospy.logwarn(f"Position ({x:.2f}, {y:.2f}) is out of bounds!")
            return True  # Consider out-of-bounds as a collision

        # Define the neighborhood search window
        # for i in range(-neighborhood_size, neighborhood_size + 1):
        #     for j in range(-neighborhood_size, neighborhood_size + 1):
        #         nx = x + i
        #         ny = y + j

        #         # Skip if out of bounds
        #         if nx < 0 or nx >= self.width or ny < 0 or ny >= self.height:
        #             continue  # Ignore cells outside the map

        #         # Check if the cell is occupied
        #         if self.grid[ny, nx] >= 80:  # Threshold for obstacle detection
        #             rospy.logwarn(f"Collision detected at ({x:.2f}, {y:.2f}) in neighborhood!")
        #             return True  # Collision found
        # return False  # No collision in the neighborhood

        # Check if the cell is occupied
        if self.grid[y, x] >= 80:  # Threshold for obstacle detection
            rospy.logwarn(f"Collision detected at ({x:.2f}, {y:.2f}) in neighborhood!")
            return True  # Collision found
        return False  # No collision in the neighborhood


# To manually set the goal pose
    def get_goal_pose(self):
        x = 1.0
        y = 0.0
        theta = 0.0
        return (x, y, theta)
        
    def get_start_pose(self):
        """ Gets the starting position directly from Gazebo as the pose in which the robot spawns"""

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
                if self.check_collision((x, y, 0.0)):  # Pass as a single-point trajectory
                    rospy.logwarn(f"Start position ({x:.2f}, {y:.2f}) is inside an obstacle! Finding nearest free space...")
                    x, y = self.find_nearest_free_space(x, y)
                #print(f"Start position set at: x = {x}, y =  {y}, theta = {theta}")
                #print(f"quaternion = {orientation_q}")
                (_, _, theta) = euler_from_quaternion([orientation_q.x, orientation_q.y,
                                                        orientation_q.z,
                                                        orientation_q.w])
                #print(f"Roll, pitch, yaw = {eulers}")
                #rospy.loginfo(f"Start position set at: ({x:.2f}, {y:.2f}, {theta: 2f})")
                return (x, y, theta)
            else:
                rospy.logerr("Starting position not found")
                return None
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return None

    def find_nearest_free_space(self, x, y):
        """To make sure the robot doesn't spawn inside an obstacle. If the robot is too close to an obstacle (or inside it),
        it moves it to the closest free space and returns the new position"""
        step_size = self.resolution  # Move in grid-sized steps
        max_search_radius = 10  # Limit search radius

        for radius in range(1, max_search_radius):
            for dx in [-radius, 0, radius]:
                for dy in [-radius, 0, radius]:
                    new_x, new_y = x + dx * step_size, y + dy * step_size
                    if not self.check_collision((new_x, new_y, 0.0)):  # Check if free
                        rospy.loginfo(f"Found free start position at: ({new_x:.2f}, {new_y:.2f})")
                        return new_x, new_y

        rospy.logwarn("No nearby free space found! Keeping original start position.")
        return x, y  # If no free space found, return original

    

    def print_occupancy_grid(self):
        """
        Prints the occupancy grid such that:
        - 0 is a free space,
        - 100 is an obstacle,
        - -1 is unknown.
        Also prints example pixels for free space, obstacles, and unknown regions.
        """
        print("\n\n\n\n\n\n\n\n\n\n\n\n")
        print("Occupancy grid map:")
        for row in reversed(self.grid):
            print(" ".join(map(str, row)))
        print("\n\n\n\n\n\n\n\n\n\n\n\n")
        example_free = None
        example_obstacle = None
        example_unknown = None
        for i in range(self.height):
            for j in range(self.width):
                value = self.grid[i][j]
                if value == 0 and example_free is None:
                    example_free = (i, j, value)
                elif value == 100 and example_obstacle is None:
                    example_obstacle = (i, j, value)
                elif value == -1 and example_unknown is None:
                    example_unknown = (i, j, value)

                # Stop once we have one example of each
                if example_free and example_obstacle and example_unknown:
                    break
            if example_free and example_obstacle and example_unknown:
                break

        print("\nExample Pixels:")
        if example_free:
            print(f"Pixel ({example_free[0]},{example_free[1]}) is FREE space, value {example_free[2]}")
        if example_obstacle:
            print(f"Pixel ({example_obstacle[0]},{example_obstacle[1]}) is an OBSTACLE, value {example_obstacle[2]}")
        if example_unknown:
            print(f"Pixel ({example_unknown[0]},{example_unknown[1]}) is UNKNOWN, value {example_unknown[2]}")
    
    # Conversion from gazebo coordinates to world coordinated and viceversa
    def world_to_map(self, x, y):
        """Converti coordinate del mondo (metri) in coordinate della mappa (celle)"""
        grid_x = int((x - self.origin.position.x) / self.resolution)
        grid_y = int((y - self.origin.position.y) / self.resolution)
        return (grid_x, grid_y)
    
    def map_to_world(self, grid_x, grid_y):
        """Converti coordinate della mappa (celle) in coordinate del mondo (metri)"""
        x = grid_x * self.resolution + self.origin.position.x
        y = grid_y * self.resolution + self.origin.position.y
        return (x, y)
