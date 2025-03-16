# MODIFIED
#!/usr/bin/env python3
import math
import numpy as np
import random
from unicycle_kinematics import Unicycle
# MODIFIED
import matplotlib.pyplot as plt
class ACO:
    """this class implement the Ant Colony Optimization (ACO) algorithm to optimize paths for RRT*, 
    focusing on reducing path length and improving smoothness. 
    
    Main method:
    -optimize(): Return an optimized path starting form the given 'best_path' using an Ant Colony Optimization strategy.

    """
    
    def __init__(self, map, best_path, vertices, unicycle, alpha=1, beta=2, evaporation_ratio=0.5, num_ants=10, iterations=5):
        self.best_path = best_path
        self.unicycle = unicycle
        self.vertices = vertices 
        self.goal = self.vertices[-1]
        self.alpha = alpha
        self.beta = beta
        self.evaporation_ratio = evaporation_ratio
        self.num_ants = num_ants
        self.iterations = iterations
        self.pheromone = np.zeros((len(self.vertices), len(self.vertices)))         #initialized at zero
        self.pheromone_deposited = np.zeros((self.num_ants, len(self.vertices), len(self.vertices)))
        self.heuristic = self.compute_heuristic()
        self.map = map

    def compute_heuristic(self):
        num_vertices = len(self.vertices)
        heuristic = np.zeros(num_vertices)
        for j, xj in enumerate(self.vertices):
            dist = math.sqrt((xj[0] - self.goal[0])**2 + (xj[1] - self.goal[1])**2)
            heuristic[j] = 1 / dist if dist != 0 else 1000
        return heuristic

    def generate_path(self):
        """this method generate a path for each ant and store them in the paths list together with the relative path_costs"""
        start_idx = 0
        goal_idx = len(self.vertices) -1 
        paths = []
        path_costs = []

        for _ in range(self.num_ants):
            path = [start_idx]                                              # list of index
            while path[-1] != goal_idx:
                current = path[-1]
                probabilities = self.compute_probabilities(current) 
                next_vertex = np.random.choice(range(len(self.vertices)), p=probabilities) 
                path.append(next_vertex)

            cost_and_angles = self.compute_path_cost_and_angles(path)
            paths.append(path)
            path_costs.append(cost_and_angles)

        return paths, path_costs

    def compute_probabilities(self, current):
        """returns the probabilities used for selecting the next vertex"""
        probabilities = np.zeros(len(self.vertices), dtype=np.float64)
        idx_allowed = self.allowed(self.vertices[current])
        for j in idx_allowed:
            probabilities[j] = (self.pheromone[current][j] ** self.alpha if self.pheromone[current][j] != 0 else 1) * (self.heuristic[j] ** self.beta) 
        probabilities /= probabilities.sum()
        if np.isnan(probabilities).any():
            next_idx = current+1
            print('next_idx',next_idx)
            q_traj = self.unicycle.plan_trajectory(self.vertices[current], self.vertices[next_idx])['points']
            print('q_traj', q_traj)                                                 #
            if not self.map.collision_free(q_traj):                                 #!!!!!!!!CORREGGI PROBLEMI CON ACO!!!!!!!!
                print('NOT COLLISION FREEE')                                        #

            raise ValueError(f"""Error: probabilities contain NaN values!
            PROBABILITIES IS {probabilities}                                        
            current: {current}
            idx_allowed: {idx_allowed}""")
        return probabilities
    
    def allowed(self, x_current):
        """return a list containing the next indices allowed for the current ant"""
        idx_allowed = []
        for j,xj in enumerate(self.vertices):
            if j>self.vertices.index(x_current):                            # set as idx_allowed only the indices of next nodes to the goal
                q_traj = self.unicycle.plan_trajectory(x_current, xj)['points']
                if self.map.collision_free(q_traj):                         # if they are collision free
                    idx_allowed.append(j)
        return idx_allowed

    def compute_path_cost_and_angles(self, path):
        """returns the path length of ant k and the relative amount of steering angles accumulated during the path"""
        for i in range(len(path) - 1):
            v1 = self.vertices[path[i]]
            v2 = self.vertices[path[i+1]]

            cost, smooth  = self.unicycle.get_path_length_and_smoothness(v1, v2)
        return cost + smooth
       
    def update_pheromone(self, paths, path_costs):
        """this method update the pheromone tau_ij after one iteration of the algorithm (after all the ants reached the goal in one iteration)"""
        self.pheromone *= (1 - self.evaporation_ratio)
        self.pheromone_deposited = np.zeros((self.num_ants, len(self.vertices), len(self.vertices)))
        k=0
        for path, cost_and_angles in zip(paths, path_costs):
            for i in range(len(path) - 1):
                self.pheromone_deposited[k][path[i]][path[i+1]] = 1 / (cost_and_angles)#[0] + cost_and_angles[1])
            k+=1
        self.pheromone += self.evaporation_ratio * self.pheromone_deposited.sum(axis=0)

    def optimize(self):
        """this method optimize the given path to the class and return a sequence of edges connecting the nodes"""
        best_path = None
        best_cost = float('inf')

        for _ in range(self.iterations):
            paths, path_costs = self.generate_path()
            self.update_pheromone(paths, path_costs)
            min_cost = min(path_costs)
            if min_cost < best_cost:
                best_cost = min_cost
                best_path = paths[np.argmin(path_costs)]
        return self.return_a_path(best_path)
    
    def return_a_path(self, path):
        edge=[]
        for i in range(len(path) - 1):
            v1 = self.vertices[path[i]]
            v2 = self.vertices[path[i+1]]
            traj = self.unicycle.plan_trajectory(v1, v2)['points']
            edge.append(traj)
        return edge


class RRTStar():
    """This class implement the RRT*  algorithm with final Ant Colony Optimization of the path including kinodynamics
        requires a map representing the configuration space of the robot to which it is applied in which are defined:
        map.start: The start configuration of the robot.
        map.goal: The goal configuration of the robot.
        map.width: The width of the map.
        map.height: The height of the map.
        map.collision_free(q1,q2): A method of the map returning True if the segment joining q1(tuple:(x,y,theta)) and q2
                                   is collision free, or False otherwise.

        Main methods:
        -update_tree(join_start): To be executed in a loop. It represents a step in the RRT* algorithm.
                                  Use join_start= True for joining the start configuration in the last iteration of the loop.
        -output_best_path(): Compute the best path connecting the start with the goal.
                             After the execution the best path obtained so far is stored in self.best_path.
        -aco_optimization(): Execute the ACO algorithm to optimize the best_path in terms of path length and path smoothness.
        
        Improvement: MOD-RRT* (for handling dynamic obstacles)
        Main method:
        -path_replanning(start): It executes a while loop in which attempt to follow the preplanned path (given by the implemented RRT* off-line method)
                                 If a new obstacle is detected on the path this method replans a path in order to avoid it according to 
                                 Pareto optimal startegy with objectives next node length (distance from current node + next node cost) and steering angle.
        
        """
    # MODIFIED
    MAX_ITER = 2000                # fixed max number of iterations that the algorithm does
    DELTA = 0.65                  # DELTA in [0,1]: distance of x_new generation starting from x_near
    DIS = 300                     # Euclidean distance used in NeighborNodes
                                  # |-> Augmenting this hyperparameter will slow the algorithm because it has to check for an higher number of neighbors

    def __init__(self, map): 
        self.vertices = []
        self.edges = []
        
        self.unicycle = Unicycle()
        self.parent = None
        self.cost = []

        #set the map
        self.map = map                           # theta is taken as the orientation w.r.t to the x axis (CounterClockwise)
        # MODIFIED
        self.start = map.get_start()                   # start config. of the robot: q=(x,y,theta)
        self.goal = map.get_goal()                     # goal config. of the robot: q=(x,y,theta)

        
        self.map_width = map.width
        self.map_height = map.height

        #insert the goal 
        self.vertices.append(self.start)         # decide the direction of expansion of the tree
        self.cost.append(0)

        #initialize best path
        self.best_path = []
        self.best_vertices = []

    def rand_sample(self):
        """this method generate a random sample with uniform probability distribution in the map"""
        x = random.randint(0, self.map_width-1)
        y = random.randint(0, self.map_height-1)
        theta = random.uniform(-math.pi/2, math.pi/2)
        return (x, y, theta)

    def compute_q_near(self, q_rand):
        """this method compute the nearest configuration to q_rand"""
        distances = []
        for q in self.vertices:
            distance = self.compute_distance(q,q_rand)
            distances.append(distance)
        nearest_index = distances.index(min(distances))
        return self.vertices[nearest_index]
    
    def compute_distance(self, q1,q2):
        """ this method compute the distance between q1 and q2 using control points"""
        x1 = q1[0]
        y1 = q1[1]
        control_point_x1 = x1 + 15 * math.cos(q1[2])
        control_point_y1 = y1 + 15 * math.sin(q1[2])

        x2 = q2[0]
        y2 = q2[1]
        control_point_x2 = x2 + 15 * math.cos(q2[2])
        control_point_y2 = y2 + 15 * math.sin(q2[2])

        distance = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2) +\
            math.sqrt((control_point_x1 - control_point_x2) ** 2 + (control_point_y1 - control_point_y2) ** 2)
        return distance
    
    def steer(self, q_near, q_rand):
        """this method returns x_new as the configuration in the direction starting from x_near to x_rand scaled by DELTA""" 
        direction = ((q_rand[0] - q_near[0]), (q_rand[1] - q_near[1]))
        q_steering = (q_near[0] + direction[0] * self.DELTA, q_near[1] + direction[1] * self.DELTA, q_rand[2])
        return q_steering
    
    def compute_q_new(self, join_q = None):
        """this method consists in the forward step of the algorithm:
        -generates a random config.
        -compute q_near
        -return q_new in the directiofrom q_near to q_rand scaled by DELTA"""
        if not join_q:
            q_rand = self.rand_sample()
        else:
            q_rand = join_q
        q_nearest = self.compute_q_near(q_rand)
        q_new = self.steer(q_nearest, q_rand)
        return q_new, q_nearest
    
    def compute_cost(self, q1, q_traj):
        """return the cost of q_2 starting from q1 and following the trajectory q_traj"""
        index = self.vertices.index(q1)
        # cost = self.cost[index]
        # q_prev = q_traj[0]
        # for q in q_traj[1:]:
        #     cost += math.sqrt((q_prev[0]-q[0]) ** 2 + (q_prev[1]-q[1]) ** 2)
        #     q_prev = q
        cost = self.cost[index] + self.unicycle.get_arc_length(q_traj)
        return cost

    def NeighborNodes(self, q_new, dist):
        """compute the set of neighbors to q_new according to the hyperparameter DIS"""
        X = []
        for x in self.vertices:
            distance = self.compute_distance(x,q_new)
            if distance < dist:
                X.append(x)
        return X

    def update_tree(self, join_goal = False):
        """this method represent execute the path_generation function in the RRT* algorithm """
        if join_goal:
            q_new = self.goal
            q_nearest = self.compute_q_near(q_new)
        else:
            q_new,q_nearest = self.compute_q_new()

        q_traj = self.unicycle.plan_trajectory(q_nearest, q_new)['points']
        
        if not self.map.collision_free(q_traj):
            #print('collision detected, remove current q_new')
            if join_goal:
                return 'No path exist'
            return 'continue'
        self.parent = q_nearest                                             # set q_nearest as parent

        cost_q_new = self.compute_cost(q_nearest, q_traj)                   # Cost(q_new) <- Cost(q_nearest) + ArcLenght(q_nearest,q_new)
        self.cost.append(cost_q_new)
        Q_near = self.NeighborNodes(q_new, self.DIS)                        # compute the set of neighbors

        # Used for returning the best parent of q_new
        for x_near in Q_near:                                               # find the best parent for joining q_new
            self.rewire(x_near, q_new, cost_q_new)
        
        self.vertices.append(q_new)                                         # add q_new to the vertices list
        if self.parent != q_nearest:
            q_traj = self.unicycle.plan_trajectory(self.parent, q_new)['points']
        self.edges.append(q_traj)                                           # add the edge connecting the best parent with q_new

        # Used for checking if q_new can be the best parent for its q_near
        for q_near in Q_near:                                               # rewire the neighborhood of q_new for replacing the parent of q_near with q_new if this reduce the cost of q_near 
            x_parenttemp, q_traj_q_new_q_near = self.rewire(q_new, q_near, parenttemp=True) # inverted order for q_new, q_near
            if x_parenttemp == q_new:
                self.remove_old_edge_of_q_near(q_near)
                self.edges.append(q_traj_q_new_q_near)


    def rewire(self, q1, q2, cost_q2=None, parenttemp=False):
        """this method rewire q2 with q1 if this reduce the cost of q2"""
        q_traj_q1_q2 = self.unicycle.plan_trajectory(q1, q2)['points']
        if self.map.collision_free(q_traj_q1_q2):
            new_cost_q2 = self.compute_cost(q1, q_traj_q1_q2)
            index = -1                                              # last index of the self.cost list
            if not cost_q2:                                         # if the param cost_q2 is not passed, find the index and the cost corresponding to q2
                index = self.vertices.index(q2)
                cost_q2 = self.cost[index]

            if new_cost_q2 < cost_q2:                               # if ArcLenght(q1,q2) + cost(q1) < cost(q2)
                self.cost[index] = new_cost_q2                      # replce cost(q2)
                if parenttemp:
                    return q1, q_traj_q1_q2
                self.parent = q1
        return None,None

    def remove_old_edge_of_q_near(self, q_near):
        """Remove the element which contains q_near as the last value in the edges list (child)."""
        for edge in self.edges:
            if (round(edge[-1][0],2) == round(q_near[0],2)) and (round(edge[-1][1],2) == round(q_near[1],2)):  
                self.edges.remove(edge)  
                break

    def output_best_path(self):
        """this method compute the best_path from the start to the goal"""
        done= False
        self.child = self.goal[:-1] 
        i=0
        while not done:
            #print('child', self.child)
            for edge in self.edges:
                #print('edge', edge[-1])
                if (round(edge[-1][0],2) == round(self.child[0],2)) and (round(edge[-1][1],2) == round(self.child[1],2)):  # edge[-1] == self.child:  
                    #self.best_vertices.append([edge[-1],edge[0]])        # append edges from start to goal
                    self.best_path.append(edge)
                    self.child = edge[0]
                    break
            if self.child == self.start[:-1]:
                done = True
            i+=1
            if i==1000:                                             # break the cycle to prevent infinite loop
                print('escape the cycle... something went wrong...') 
                break

        self.best_path = self.best_path[::-1]
        # Ant Colony Optimization
        #self.aco_optimization()

    def aco_optimization(self):
        """optimize the best_path using ACO. 
        It must be called after computing the best_path with output_best_path method"""
        aco = ACO(self.map, self.best_path, self.get_vertices_from_best_path(), self.unicycle)
        self.best_path = aco.optimize()

    def get_vertices_from_best_path(self):
        vertices = [self.start]
        for path in self.best_path:
            for vertex in self.vertices:
                if (round(vertex[0],2) == round(path[-1][0],2)) and (round(vertex[1],2) == round(path[-1][1],2)):
                    vertices.append(vertex)
        print('vertices', vertices)
        return vertices
            
    def path_replanning(self, x_current):
        """This is the main method for MOD-RRT*
        Execute the dynamic replanning of the path in a loop until it reaches the goal.
        the optimization is based on minimizing the objectives of next node length and steering angle using the Pareto dominance method."""
        count = 0
        x_prev=[x_current]
        self.replanned_best_path = []
        self.sample_best_path()                                 # this is required to discretize the best path
        while x_current != self.goal:

            next_path_node = self.compute_next_path_node(x_current, count)

            if self.find_obstacle(x_current, next_path_node):
                x_current = self.get_current_position(x_current)

                X_candi = []
                X_near = self.NeighborNodes(x_current, dist=50)
                for x_near in X_near:
                    if x_near in x_prev: continue
                    if self.map.collision_free(x_current, x_near):
                        # Compute length
                        length = self.compute_cost(x_near, x_current)
                        # compute angle theta
                        alpha = math.atan2(next_path_node[1]-x_current[1], next_path_node[0]-x_current[0])
                        beta = math.atan2(x_near[1]-x_current[1], x_near[0]-x_current[0])
                        theta = abs(beta-alpha)
                        theta = math.degrees(theta)

                        X_candi.append((x_near, length, theta))

                if not X_candi: raise ValueError("No neighbor candidates were found. Consider increasing the number of iterations.")
                x_next = self.Pareto(X_candi)

            # normal execution if doesn't find the obstacle
            else:
                x_next = next_path_node

            count +=1
            # print('x_next:',x_next)
            self.replanned_best_path.append((x_current,x_next))
            x_current = x_next
            x_prev.append(x_current)

    def compute_next_path_node(self, x_current, idx):
        """This method returns the nearest next node of the pre-computed path to x_current"""
        min_distance =100000
        next_pathe_node = self.best_path[-1][1]
        for edge in self.best_path[idx:]:
            distance = self.compute_distance(x_current,edge[1])
            if  distance < min_distance:
                min_distance = distance 
                next_pathe_node = edge[1]
        return next_pathe_node
    
    def sample_best_path(self, render = False):
        """This method is used to discretize the best path and compute internal nodes cost.
            Is possible to visualize the discretized points if setting render to True. """
        x_current = self.best_path[-1][-1]
        count=-1
        path = []
        while x_current != self.start:
            next_best_path = self.best_path[count][0]
            distance = math.sqrt( (x_current[0] - next_best_path[0]) ** 2 + (x_current[1] - next_best_path[1]) ** 2)  

            if distance > 20:
                direction = ((next_best_path[0] - x_current[0])/ distance, (next_best_path[1] - x_current[1])/distance)
                next_path_node = (x_current[0] + (direction[0] * 20), x_current[1] + (direction[1] * 20))

                if render:
                    self.map.add_point(next_path_node, color=(0,255,255))
                    self.map.draw()
                 
                cost = self.compute_cost(x_current , next_path_node)
                self.vertices.append(next_path_node)
                self.cost.append(cost)
            else:
                next_path_node = next_best_path
                count-=1

            path.insert(0, [next_path_node, x_current])
            x_current = next_path_node
        self.best_path = path

    def find_obstacle(self, x_current, x_next):
        """Should be a laser scan reading, we need a conic area to exclude the nodes within it.
        This represents a simplified version."""
        if not self.map.collision_free(x_current, x_next):
            return True
        else: 
            return False

    def get_current_position(self, x_current):
        """implement a localization method"""
        return x_current

    def Pareto(self, X_candi):
        """This method implement the Pareto dominance function to be used in the path_replanning method."""
        # Extract objectives: length and theta
        objectives = np.array([(length, theta) for _, length, theta in X_candi])

        n = len(X_candi)
        in_degree = np.zeros(n, dtype=int)
        out_degree = np.zeros(n, dtype=int)
        
        # Compute Pareto dominance
        for i in range(n):
            for j in range(n):
                if i != j:
                    # Check if node i dominates node j
                    if all(objectives[i] <= objectives[j]) and any(objectives[i] < objectives[j]):
                        out_degree[i] += 1
                        in_degree[j] += 1

        indices_with_indegree_0 = [i for i, degree in enumerate(in_degree) if degree == 0]

        out_degrees_for_indegree_0 = [out_degree[i] for i in indices_with_indegree_0]
        max_out_degree = max(out_degrees_for_indegree_0)
        indices_with_max_outdegree = [
        indices_with_indegree_0[i]
        for i, degree in enumerate(out_degrees_for_indegree_0)
        if degree == max_out_degree
        ]

        if len(indices_with_max_outdegree) != 1:
            x_rand = X_candi[random.choice(indices_with_max_outdegree)][0]
            return x_rand

        return X_candi[indices_with_max_outdegree[0]][0]
    
    # MODIFIED
    def visualize_rrt(rrt_nodes, edges, path=None, obstacles=None):
        plt.figure(figsize=(10, 5))

        # Plot obstacles if available
        if obstacles is not None:
            plt.imshow(obstacles, cmap='gray', origin='lower', extent=[0, 10, 0, 10])

        # Plot the sampled points (nodes)
        for node in rrt_nodes:
            plt.scatter(node[0], node[1], color='blue', s=5)  # Small blue dots

        # Plot the edges (connections)
        for edge in edges:
            plt.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], color='gray', linewidth=0.5)

        # Plot the final path in red
        if path:
            for i in range(len(path) - 1):
                plt.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], color='red', linewidth=2)

        plt.xlabel("X (meters)")
        plt.ylabel("Y (meters)")
        plt.title("RRT* Path Planning")
        plt.grid(True)
        plt.show()

    
    