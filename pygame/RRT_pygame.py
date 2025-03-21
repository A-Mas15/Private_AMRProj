import random 
import math
import numpy as np

from unicycle_kinematics import Unicycle

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
        # for DEBUG
        if np.isnan(probabilities).any():
            next_idx = current+1
            print('next_idx',next_idx)
            q_traj = self.unicycle.plan_trajectory(self.vertices[current], self.vertices[next_idx])['points']
            print('q_traj', q_traj)                                                 
            if not self.map.collision_free(q_traj):                                
                print('NOT COLLISION FREEE')                                        

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
                self.pheromone_deposited[k][path[i]][path[i+1]] = 1 / (cost_and_angles)
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
        map.check_collision(v): ....

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
    
    MAX_ITER = 500                # fixed max number of iterations that the algorithm does
    DELTA = 0.65                  # DELTA in [0,1]: distance of x_new generation starting from x_near
    DIS = 300                     # distance used in NeighborNodes
                                  # |-> Augmenting this hyperparameter will slow the algorithm because it has to check for an higher number of neighbors

    def __init__(self, map): 
        self.vertices = []
        self.edges = []
        
        self.unicycle = Unicycle()
        self.parent = None
        self.cost = []

        # set the map
        self.map = map                           # theta is taken as the orientation w.r.t to the x axis (CounterClockwise)
        self.start = map.start                   # start config. of the robot: q=(x,y,theta)
        self.goal = map.goal                     # goal config. of the robot: q=(x,y,theta)
        self.map_width = map.width
        self.map_height = map.height

        # insert the goal 
        self.vertices.append(self.goal)         # decide the direction of expansion of the tree
        self.cost.append(0)

        # initialize best path
        self.best_path = []
        self.best_vertices = []

        # used in replanning
        self.x_prev = []

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

    def update_tree(self, join_start = False):
        """this method represent execute the path_generation function in the RRT* algorithm """
        if join_start:
            q_new = self.start
            q_nearest = self.compute_q_near(q_new)
        else:
            q_new,q_nearest = self.compute_q_new()

        q_traj = self.unicycle.plan_trajectory(q_new, q_nearest)['points']
        
        if not self.map.collision_free(q_traj):
            #print('collision detected, remove current q_new')
            if join_start:
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
            q_traj = self.unicycle.plan_trajectory(q_new, self.parent)['points']
        self.edges.append(q_traj)                                           # add the edge connecting the best parent with q_new

        # Used for checking if q_new can be the best parent for its q_near
        for q_near in Q_near:                                               # rewire the neighborhood of q_new for replacing the parent of q_near with q_new if this reduce the cost of q_near 
            x_parenttemp, q_traj_q_near_q_new = self.rewire(q_new, q_near, parenttemp=True) # inverted order for q_new, q_near
            if x_parenttemp == q_new:
                self.remove_old_edge_of_q_near(q_near)
                self.edges.append(q_traj_q_near_q_new)

    def rewire(self, q1, q2, cost_q2=None, parenttemp=False):
        """this method rewire q2 with q1 if this reduce the cost of q2"""
        q_traj_q1_q2 = self.unicycle.plan_trajectory(q2, q1)['points']
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
        """Remove the element which contains q_near as the first value in the edges list (child)."""
        for edge in self.edges[:]:
            if edge[0] == q_near: 
                self.edges.remove(edge)  
                break
    
    def output_best_path(self):
        """this method compute the best_path from the start to the goal"""
        done= False
        self.child = self.start 
 
        while not done:
            flag = False
            for edge in self.edges:
                if edge[0] == self.child:  
                    self.best_path.append(edge)
                    self.child = edge[-1]
                    flag = True
                    break
            if edge[-1] == self.goal:
                done = True

            # FOR DEBUGGING
            if flag == False:
                print(f'current: {round(self.child[0],0)}, {round(self.child[1],0)}' )
                for edge in self.edges:
                    print(f'{round(edge[-1][0],0)}, {round(edge[-1][1],0)}')
                raise ValueError('escape the cycle... something went wrong...')

        # Ant Colony Optimization
        self.aco_optimization()

    def aco_optimization(self):
        """optimize the best_path using ACO. 
        It must be called after computing the best_path with output_best_path method"""
        aco = ACO(self.map, self.best_path, self.get_vertices_from_best_path(), self.unicycle)
        self.best_path = aco.optimize()
        self.best_vertices = self.get_vertices_from_best_path()

    def get_vertices_from_best_path(self):
        """Returns the vertices of the best path"""
        vertices = [self.start]
        for path in self.best_path:
            vertices.append(path[-1])
        return vertices

    def replan_path(self, x_current, next_path_node):
        """This is the main method for MOD-RRT*
        Execute the dynamic replanning of the path in order to find the next collisionfree edge where to move.
        the optimization is based on minimizing the objectives of next node length and steering angle using the Pareto dominance method."""
        self.x_prev.append(x_current)
        X_candi = []
        X_near = self.NeighborNodes(x_current, dist=300)
        for x_near in X_near:
            if x_near in self.x_prev: continue
            q_traj, _, smoothness_cost = self.unicycle.get_path_length_and_smoothness(x_current, x_near, return_q_traj= True)
            if self.map.collision_free(q_traj):
                # Compute length
                length = self.compute_cost(x_near, q_traj)
                # Compute deviation angle from optimal trajectory
                alpha = math.atan2(next_path_node[1]-x_current[1], next_path_node[0]-x_current[0])
                beta = math.atan2(x_near[1]-x_current[1], x_near[0]-x_current[0])
                theta = abs(beta-alpha) #+ smoothness_cost

                X_candi.append((x_near, length, theta))

        if not X_candi: raise ValueError("No neighbor candidates were found. Consider increasing the number of iterations.")
        x_next = self.Pareto(X_candi)
        #print('x_next PARETO:',x_next)

        next_edge = self.unicycle.plan_trajectory(x_current, x_next)['points']
        return next_edge, x_next

    def compute_next_path_node(self, x_current, executed_path):
        """This method returns the next node of the pre-computed path from x_current"""

        direct_path_to_goal = self.unicycle.plan_trajectory(x_current, self.goal)['points']
        if self.map.collision_free(direct_path_to_goal):
            return direct_path_to_goal, self.goal

        latest_index = -1

        for item in executed_path:
            if item in self.best_vertices:
                latest_index = self.best_vertices.index(item)

        next_path_node = self.best_vertices[latest_index + 1] if latest_index + 1 < len(self.best_vertices) else None

        if next_path_node == None:
            raise ValueError('index out of bounds') 
        
        while self.map.check_collision(next_path_node):
            if latest_index < len(self.best_vertices) -1:
                latest_index+=1
                next_path_node = self.best_vertices[latest_index + 1]
            else:
                raise ValueError('Goal is in collision')
            
        next_edge = self.unicycle.plan_trajectory(x_current, next_path_node)['points']
        return next_edge, next_path_node
        
    def find_obstacle(self, x_current, x_next):
        """OLD VERSION:Should be a laser scan reading, we need a conic area to exclude the nodes within it.
        This represents a simplified version.
        FOR THE NEW VERSION CHECK IN THE MAIN.PY"""
        q_traj = self.unicycle.plan_trajectory(x_current, x_next)['points']
        # assuming to have knowledge of all the map (if you have a laser scanner you have to change this!)
        if not self.map.collision_free(q_traj):
            return True
        else:
            return False

    def update_tree_structure(self):
        # Remove nodes in collision
        for v in self.vertices[:]:
            if self.map.check_collision(v):
                idx = self.vertices.index(v)
                # remove nodes in collision
                self.vertices.pop(idx)
                self.cost.pop(idx)
         
        # Remove edges in collision
        orphans = []
        for edge in self.edges[:]:
            if not self.map.collision_free(edge):
                orphan = edge[0]
                if orphan in self.vertices:
                    idx = self.vertices.index(orphan)                     
                    self.cost[idx] = 10000000                           # need to reassign the cost of the orphan node before rewire
                    orphans.append(orphan)

                self.edges.remove(edge)

        # REWIRE ORPHAN NODES
        # remove from Q_near the children of orph
        Q_children = self.get_children_of_orph(orphans)

        # rewire the orphans
        self.rewire_orphan_node(orphans, Q_children, reconnection_range = self.DIS)

        # rewire their nodes
        self.rewire_orphan_node(Q_children, [], reconnection_range = self.DIS)


    def rewire_orphan_node(self, orphans, Q_children, reconnection_range):
        rewired_nodes = []

        for orph in orphans:
            Q_near = self.NeighborNodes(orph, reconnection_range)

            to_remove = list(set(orphans) - set(rewired_nodes))
            Q_cands = list(set(Q_near) - set(Q_children)- set(to_remove))

            for q_near in Q_cands:
                parent,q_traj = self.rewire(q_near, orph, parenttemp=True)
                if parent == q_near:
                    self.remove_old_edge_of_q_near(orph)   
                    self.edges.append(q_traj)
            
            # Error management
            idx = self.vertices.index(orph)                     
            if self.cost[idx]==10000000: 
                """this happens if was not possible to do the rewire on orph because of small neighborhood 
                or all connections with orph are in collision.
                In this case:
                - romove orph from the vertices and the connections with it"""

                for edge in self.edges[:]:
                    if edge[-1] == orph:
                        self.edges.remove(edge)
                
                idx = self.vertices.index(orph)
                self.vertices.pop(idx)
                self.cost.pop(idx)

            else:
                rewired_nodes.append(orph)
    
    def get_children_of_orph(self, orphans):
        """This method returns the all the children of the orphans list selected.
        Moreover replace the old cost of each child of the orphan with a default high cost (e.g., 10000000)"""
        Q_children = []  
        Q_edges = []
        for orph in orphans:
            queue = [orph]  

            while queue:
                parent = queue.pop(0)  
                for edge in self.edges:
                    if edge[-1] == parent: 
                        child = edge[0]
                        if child not in Q_children:  
                            idx = self.vertices.index(child)                     
                            self.cost[idx] = 10000000
                            Q_children.append(child)
                            queue.append(child)  
                            Q_edges.append(edge)
                            
                            if child not in self.vertices: raise ValueError('IMPOSSIBLE ERROR')
        return Q_children

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