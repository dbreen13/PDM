
import numpy as np
import pygame as py

from map_environment.environment import Environment
from map_environment.obstacles import Rectangle
from global_planner.rrt import RRT


if __name__ == '__main__':
    # Program variables
    v_shelf_1 = Rectangle(np.array([30, 50]), np.array([32, 200]))
    v_shelf_2 = Rectangle(np.array([90, 50]), np.array([32, 200]))
    v_shelf_3 = Rectangle(np.array([150, 50]), np.array([32, 200]))
    v_shelf_4 = Rectangle(np.array([210, 50]), np.array([32, 200]))
    v_shelf_5 = Rectangle(np.array([270, 50]), np.array([32, 200]))
    v_shelf_6 = Rectangle(np.array([300, 290]), np.array([32, 200]))
    v_shelf_7 = Rectangle(np.array([360, 290]), np.array([32, 200]))
    v_shelf_8 = Rectangle(np.array([420, 290]), np.array([32, 200]))
    v_shelf_9 = Rectangle(np.array([480, 290]), np.array([32, 200]))
    v_shelf = (v_shelf_1, v_shelf_2, v_shelf_3, v_shelf_4, v_shelf_5, v_shelf_6, v_shelf_7, v_shelf_8, v_shelf_9)

    h_shelf_1 = Rectangle(np.array([330, 50]), np.array([200, 32]))
    h_shelf_2 = Rectangle(np.array([330, 110]), np.array([200, 32]))
    h_shelf_3 = Rectangle(np.array([330, 170]), np.array([200, 32]))
    h_shelf_4 = Rectangle(np.array([330, 230]), np.array([200, 32]))
    h_shelf_5 = Rectangle(np.array([30, 280]), np.array([240, 32]))
    h_shelf_6 = Rectangle(np.array([30, 340]), np.array([240, 32]))
    h_shelf_7 = Rectangle(np.array([30, 400]), np.array([240, 32]))
    h_shelf_8 = Rectangle(np.array([30, 460]), np.array([240, 32]))

    h_shelf = (h_shelf_1, h_shelf_2, h_shelf_3, h_shelf_4, h_shelf_5, h_shelf_6, h_shelf_7, h_shelf_8)

    # Environment variables
    c_space_dimension_boundaries = np.array([550, 550], dtype='int')
    c_space_obstacle_points = np.array([*v_shelf, *h_shelf])

    starting_point = np.array([20, 20])
    car_dimensions = np.array([6, 7.5])

    # Initiate pygame and give permission
    py.init()

    # Create the global rrt planner
    rrt_global_planner = RRT(c_space_dimension_boundaries, c_space_obstacle_points)

    # Create the environment map
    environment_map = Environment(c_space_dimension_boundaries, c_space_obstacle_points, starting_point, car_dimensions)

    # Change origin pygame
    # display_surface = pygame.display.get_surface()
    # display_surface.blit(pygame.transform.flip(display_surface, False, True), dest=(0, 0))

    # define origin for the RRT algorithm
    py.draw.circle(environment_map.screen, environment_map.RGB_GREEN_CODE, starting_point, 3)

    # Main loop
    running = True
    while running:

        # Look at every event in the queue
        for event in py.event.get():

            if event.type == py.MOUSEBUTTONUP:
                ending_point = np.array(py.mouse.get_pos())

                with environment_map.in_progress():
                    rrt_nodes, rrt_edges = rrt_global_planner.build_roadmap(starting_point, ending_point, intermediate_goal_check= True)
                    shortest_path_edges = rrt_global_planner.generate_shortest_path(starting_point, ending_point)
                    shortest_path_coordinates = rrt_global_planner.coordinates_from_shortest_path(rrt_nodes, shortest_path_edges)

                # draw the resulting graph and shortest path nodes and edges
                environment_map.draw_path(rrt_nodes, rrt_edges, shortest_path_edges)

                for coordinate in shortest_path_coordinates:
                    environment_map.draw_vehicle(coordinate[0], coordinate[1], 31.4159265)
                    py.time.wait(1000)

            if event.type == py.KEYDOWN:
                # Was it the Escape key? If so, stop the loop.
                if event.key == py.K_ESCAPE:
                    running = False

            # Did the user click the window close button? If so, stop the loop.
            elif event.type == py.QUIT:
                running = False
