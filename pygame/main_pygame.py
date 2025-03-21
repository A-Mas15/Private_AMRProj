from map import Map
from RRT import RRTStar

import pygame
import time


def main():
    #map initialization
    map_width, map_height = 800, 600
    map = Map(map_width, map_height)
    map.generate_map()

    #RRT initialization
    rrt = RRTStar(map)
        
    running = True
    join_start = False
    clock = pygame.time.Clock()

    #OFF LINE PATH GENERATION
    for iter in range(rrt.MAX_ITER):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        if running == False: break
        
        #JOIN THE GOAL CONFIG AFTER MAX_ITER
        if iter >= rrt.MAX_ITER -1:                                         # if it is the last iteration, try to join the goal
            join_start =True

        string = rrt.update_tree(join_start= join_start)
        if string == 'continue':
            continue
        elif string == 'No path exist':                                     # if after the last iteration is not possible to join the start end the program
            return print(f'No possible paths have been found in {rrt.MAX_ITER} iterations')

        # Update logic of the map
        q_new = rrt.vertices[-1]
        if not join_start:
            map.add_point(q_new)
        map.add_primitives(rrt.edges.copy())

        # Redraw the map
        map.draw()

        # Set the speed of refresh of the map
        clock.tick(600000)

    # COMPUTE THE BEST PATH AND DRAW IT
    rrt.output_best_path()                 
    map.add_best_path(rrt.best_path , color=(0, 255, 255))

    map.draw()

    # ADD NEW OBSTACLES
    for i in range(2):
        map.generate_dynamic_obstacle()
    map.draw()

    print('best_path:',rrt.best_vertices)
    x_current = map.start

    current = x_current                 # Only here. Not needed in real time (we get pose from slam)
    # x_prev = x_current
    # dx_prev = 0
    # dt = rrt.unicycle.dt
    executed_path = {'nodes':[map.start], 'edges':[]}
    # DYNAMIC RE-PLANNING
    while x_current != rrt.goal:

        # map.update_slam()                         # TO BE DONE
        # new_map = map.get_map()                   # not necessary since when passing MAP to the class RRT it refers to the reference of the object (it updates when the original object updates)

        rrt.update_tree_structure()                 # or move it inside if FindObstacle()

        x_current = map.get_current_pose(current)
        #print('x_current', x_current)
        next_edge, next_node = rrt.compute_next_path_node(x_current, executed_path['nodes']) 
        #print('next_node', next_node)
        #print('next_edge', next_edge[-1])

        if FindObstacle(map, next_edge):
            next_edge, next_node = rrt.replan_path(x_current, next_node)

        current = next_node                              # Only here (we assume a perfect execution of the trajectory s.t. x_current[t+1]=x_next[t]). Not needed in real time (we send commands to the controller)
        x_current = map.get_current_pose(current)

        if executed_path['nodes'][-1] != next_node:      #if next_node not in executed_path['nodes']:
            executed_path['nodes'].append(next_node)
            executed_path['edges'].append(next_edge)
            # trajectory_to_track = next_edge
            # i = 0

        # SIMULATION ON GAZEBO
        # USE: trajectory_to_track
        # if i < len(trajectory_to_track) - 1: i += 1
        # x_d = trajectory_to_track[i] 
        # dx_d = (x_d - x_prev)/dt
        # ddx_d = (dx_d - dx_prev)/dt
        # u, v = controller.track_trajectory(x_d, dx_d, ddx_d, x_current, dx_current) # Dynamic feedback linearization (PD + fdw)
        # dx_prev = (x_current - x_prev)/dt or get_from_odometry_linear_and_angular
        # x_prev = x_current

        # display the updated structure of the tree
        map.add_primitives(rrt.edges.copy())
        map.points = []
        for v in rrt.vertices:
            map.add_point(v)
        map.add_point(next_node, color=(255,0,0))
        map.add_best_path(rrt.best_path , color=(0, 255, 255))
        map.add_best_path(executed_path['edges'] , color=(255, 0, 0))   
        map.draw()
        time.sleep(2)     
    
    print('path executed')
    for node in executed_path['nodes']:
        print(node)

    map.draw()
    print('finish')

    # Maintain the final window on the screen for testing
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if running == False: break

    pygame.quit()

def FindObstacle(map, next_edge): 
    """Information from sensors are used with SLAM to update the map, 
        so if a collision is detected a new obstacle has appeard"""
    if not map.collision_free(next_edge):
        return True
    else:
        return False

if __name__ == "__main__":
    main()