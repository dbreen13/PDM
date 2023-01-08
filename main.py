
import numpy as np
import pygame

from map_environment.environment import Environment
from map_environment.obstacles import Rectangle
from global_planner.rrt import RRT


if __name__ == '__main__':
    # Program variables
    v_shelf_1 = Rectangle(np.array([50, 50]), np.array([20, 200]))
    v_shelf_2 = Rectangle(np.array([100, 50]), np.array([20, 200]))
    v_shelf_3 = Rectangle(np.array([150, 50]), np.array([20, 200]))
    v_shelf_4 = Rectangle(np.array([200, 50]), np.array([20, 200]))
    v_shelf_5 = Rectangle(np.array([250, 50]), np.array([20, 200]))
    v_shelf = (v_shelf_1, v_shelf_2, v_shelf_3, v_shelf_4, v_shelf_5)

    h_shelf_1 = Rectangle(np.array([300, 50]), np.array([200, 20]))
    h_shelf_2 = Rectangle(np.array([300, 100]), np.array([200, 20]))
    h_shelf_3 = Rectangle(np.array([300, 150]), np.array([200, 20]))
    h_shelf_4 = Rectangle(np.array([300, 200]), np.array([200, 20]))
    h_shelf_5 = Rectangle(np.array([300, 250]), np.array([200, 20]))
    h_shelf_6 = Rectangle(np.array([300, 300]), np.array([200, 20]))
    h_shelf_7 = Rectangle(np.array([300, 350]), np.array([200, 20]))
    h_shelf_8 = Rectangle(np.array([300, 400]), np.array([200, 20]))

    h_shelf = (h_shelf_1, h_shelf_2, h_shelf_3, h_shelf_4, h_shelf_5, h_shelf_6, h_shelf_7, h_shelf_8)

    # Environment variables
    c_space_dimension_boundaries = np.array([550, 550], dtype='int')
    c_space_obstacle_points = np.array([*v_shelf, *h_shelf])

    starting_point = np.array([0, 0])

    # Initiate pygame and give permission
    pygame.init()

    # Create the global rrt planner
    rrt_global_planner = RRT(c_space_dimension_boundaries, c_space_obstacle_points)

    # Create the environment map
    environment_map = Environment(c_space_dimension_boundaries, c_space_obstacle_points)

    # Change origin pygame
    # display_surface = pygame.display.get_surface()
    # display_surface.blit(pygame.transform.flip(display_surface, False, True), dest=(0, 0))

    # define origin for the RRT algorithm
    pygame.draw.circle(environment_map.screen, environment_map.RGB_GREEN_CODE, starting_point, 3)

    # Main loop
    running = True
    while running:
        pygame.display.update()

        # Look at every event in the queue
        for event in pygame.event.get():

            if event.type == pygame.MOUSEBUTTONUP:
                ending_point = np.array(pygame.mouse.get_pos())

                with environment_map.in_progress():
                    rrt_nodes, rrt_edges = rrt_global_planner.build_roadmap(starting_point, ending_point, intermediate_goal_check= True)
                    shortest_path_edges = rrt_global_planner.generate_shortest_path(starting_point, ending_point)

                # draw the resulting graph and shortest path nodes and edges
                environment_map.draw_path(rrt_nodes, rrt_edges, shortest_path_edges)

            if event.type == pygame.KEYDOWN:
                # Was it the Escape key? If so, stop the loop.
                if event.key == pygame.K_ESCAPE:
                    running = False

            # Did the user click the window close button? If so, stop the loop.
            elif event.type == pygame.QUIT:
                running = False
