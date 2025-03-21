import pygame
import numpy as np
import random
import time
import math

from unicycle_kinematics import Unicycle

class Map:
    """Used in  the first step of the project for the implementation of the RRT* algorithm in the 2D case over the assumption of free-flying robot
    """
    def __init__(self, width, height):
        """Initialize the map with a given width and height."""
        self.width = width
        self.height = height
        self.obstacles = []  # List of rectangular obstacles
        self.points = []
        self.edges = []
        self.start_time = time.time()

        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Dynamic Map with Obstacles")

        self.start = (30, 30, 0)
        self.goal = (self.width-30, self.height-30, 0)
        
        # triangle drawing
        self.base_tri = 6
        self.tip_tri = 15

        # collision detection
        self.radius = 25

        # initialize kinematics
        #self.robot = Unicycle()

    def add_point(self,q, color =(100,100,100)):
        """Add a config to the map."""
        self.points.append((q, color))
    
    def add_primitives(self, primitive):
        """Add a list of edges (pairs of points) to the map"""
        np_array = np.array(primitive)
        self.edges = np_array[:,:, :2].tolist()
        #self.edges = primitive

    def add_best_path(self, new_edges, color =(0,0,0)):
        """Add a list of edges for the best_path (pairs of points) to the map, allowing for the selection of the color.
           Be careful: this method doesn't clean the edges list, it only appends the new edges!"""
        for edge in new_edges:
            np_array = np.array(edge)
            new_arr = np_array[:, :2].tolist()
            self.edges.append([new_arr, color])

    def add_obstacle(self, x, y, w, h, color=(0,0,0)):
        """Add a rectangular obstacle to the map."""
        self.obstacles.append((pygame.Rect(x, y, w, h), color))

    def add_random_obstacle(self):
        """Generate and add a random rectangular obstacle to the map."""
        max_width, max_height = 100, 100  # Max size of the obstacle
        min_size = 20  # Minimum size of the obstacle

        # Generate random dimensions and position
        w = random.randint(min_size, max_width)
        h = random.randint(min_size, max_height)
        x = random.randint(0, self.width - w)
        y = random.randint(0, self.height - h)

        self.add_obstacle(x, y, w, h)

    def generate_map(self):
        for col in range(0,self.width,80):
            if col<30 or col >self.width -90:
                self.add_obstacle(col,random.randint(50, self.height - 50),random.randint(20, 40),random.randint(30, 60)) #To avoid collision with start and goal
            else:
                self.add_obstacle(col,random.randint(0, self.height - 100),random.randint(20, 40),random.randint(30, 200))

    def generate_triangle(self, q):
        x,y,theta = q
        pointing_dir = [math.cos(theta), math.sin(theta), 0]           # opportunitely modified because the origin is in top left corner
        orthogonal_dir = [math.sin(theta), -math.cos(theta), 0]
        vertex1 = [x + self.base_tri/2 * orthogonal_dir[0], y + self.base_tri/2 * orthogonal_dir[1]]
        vertex2 = [x + self.tip_tri * pointing_dir[0], y + self.tip_tri * pointing_dir[1]]
        vertex3 = [x - self.base_tri * orthogonal_dir[0], y - self.base_tri * orthogonal_dir[1]]
        return [vertex1, vertex2, vertex3]
    
    def draw(self):
        """Draw the map with obstacles."""
        # Fill the screen with white (free space)
        self.screen.fill((255, 255, 255))

        # the start and goal points as triangles
        pygame.draw.polygon(self.screen, (0, 0, 255), self.generate_triangle(self.start))
        pygame.draw.polygon(self.screen, (0, 255, 0), self.generate_triangle(self.goal))

        # Draw each obstacle as a black rectangle
        for obstacle in self.obstacles:
            pygame.draw.rect(self.screen, obstacle[1], obstacle[0])

        # Draw each generated point as a black circle
        for point in self.points:
            pygame.draw.polygon(self.screen, point[1], self.generate_triangle(point[0]))

        # Draw each edge as a line
        for edge in self.edges:
            if len(edge[-1]) == 3:
                pygame.draw.lines(self.screen, color=edge[-1], points=edge[0], closed=False, width=2) 
            else:
                pygame.draw.lines(self.screen, color=(0, 0, 0), points=edge, closed=False, width=1) 
            
        # Update the display
        pygame.display.flip()

    def generate_dynamic_obstacle(self):
        """Execute additional logic or updates while the map is running."""
        # current_time = time.time()
        # if current_time - self.start_time >= 2:
        center_x = self.width // 2
        center_y = self.height // 2
        region_size = 300 #region of dynamic obstacle appearison

        for _ in range(10):
            rand_x = random.randint(center_x - region_size // 2, center_x + region_size // 2)
            rand_y = random.randint(center_y - region_size // 2, center_y + region_size // 2)

            new_obstacle = pygame.Rect(rand_x, rand_y, 50, 50)

            if not self.check_collision(new_obstacle):
                self.add_obstacle(rand_x, rand_y, 50, 50, (255, 0, 0))  
                break  

            # self.start_time = float('inf')  # Ensure the obstacle is added only once

    def run(self):
        """Run the map display and handle events."""
        running = True
        clock = pygame.time.Clock()  # To control the frame rate

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_RETURN:  # Check if Enter key is pressed: if so generate a new map
                        self.obstacles.clear()
                        self.start_time = time.time()
                        self.generate_map()


            # Update logic
            self.generate_dynamic_obstacle()

            # Redraw the map
            self.draw()

            # Cap the frame rate to 60 FPS
            clock.tick(60)

        pygame.quit()
        
    def check_collision(self, config):
        """Check if an obstacle collides with the existing others."""
        collision_ray = self.radius
        
        if isinstance(config, tuple):
            x=config[0]-collision_ray//2
            y=config[1]-collision_ray//2
            config = pygame.Rect((x,y),(collision_ray,collision_ray))

        for obstacle in self.obstacles:
            if config.colliderect(obstacle[0]):  # if there is a collision
                return True
        return False

    def collision_free(self, q_traj):
        """Check if exist a collision-free path from start to end."""
        # q_traj = self.robot.generate_primitive(q_start, q_end)

        for point in q_traj:
            # check if the point is in collision with obstacles
            if self.check_collision(point):
                return False
        return True
    
    def update_slam(self):
        # TO BE DONE
        return
    
    def get_current_pose(self, current):
        # TO BE DONE
        return current


    
# Example usage
if __name__ == "__main__":

    from unicycle_kinematics import Unicycle

    robot = Unicycle()

    map_width, map_height = 800, 600
    map = Map(map_width, map_height)
    map.generate_map()

    map.draw()
    edges=[]
    running=True
    clock = pygame.time.Clock()
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        x = random.randint(0, map.width)
        y = random.randint(0, map.height)
        theta = random.uniform(-math.pi/2, math.pi/2)
        q = (x,y,theta)

        q_finals = robot.get_primitives_final_points(q)
        Q = robot.generate_primitives(q)
        edges.append(Q[0])
        edges.append(Q[1])
        edges.append(Q[2])
        
        map.add_point(q, color=(255,0,0,))
        map.add_primitives(edges)
        map.add_point(q_finals[0],color=(255,0,0,))
        map.add_point(q_finals[1],color=(255,0,0,))
        map.add_point(q_finals[2],color=(255,0,0,))

        # Redraw the map
        map.draw()

        clock.tick(1)

    pygame.quit()