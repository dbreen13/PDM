
import numpy as np
import pygame as py

from map_environment.environment import Environment
from map_environment.obstacles import Rectangle
from map_environment.obstacles import Circle
from global_planner.rrt import RRT


if __name__ == '__main__':
    # Program variables
    v_shelf_1 = Rectangle(np.array([30, 50]), np.array([16, 200]))
    v_shelf_2 = Rectangle(np.array([70, 50]), np.array([16, 200]))
    v_shelf_3 = Rectangle(np.array([110, 50]), np.array([16, 200]))
    v_shelf_4 = Rectangle(np.array([150, 50]), np.array([16, 200]))
    v_shelf_5 = Rectangle(np.array([190, 50]), np.array([16, 200]))

    v_shelf_6 = Rectangle(np.array([300, 290]), np.array([16, 240]))
    v_shelf_7 = Rectangle(np.array([340, 290]), np.array([16, 240]))
    v_shelf_8 = Rectangle(np.array([380, 290]), np.array([16, 240]))
    v_shelf_9 = Rectangle(np.array([420, 290]), np.array([16, 240]))

    v_shelf_10 = Rectangle(np.array([90, 530]), np.array([16, 200]))
    v_shelf_11 = Rectangle(np.array([130, 530]), np.array([16, 200]))
    v_shelf_12 = Rectangle(np.array([170, 530]), np.array([16, 200]))
    v_shelf_13 = Rectangle(np.array([210, 530]), np.array([16, 200]))

    v_shelf_14 = Rectangle(np.array([500, 50]), np.array([16, 160]))
    v_shelf_15 = Rectangle(np.array([540, 50]), np.array([16, 160]))
    v_shelf_16 = Rectangle(np.array([580, 50]), np.array([16, 200]))

    v_shelf_17 = Rectangle(np.array([740, 300]), np.array([16, 200]))
    v_shelf_18 = Rectangle(np.array([780, 300]), np.array([16, 200]))
    v_shelf_19 = Rectangle(np.array([820, 300]), np.array([16, 200]))
    v_shelf_20 = Rectangle(np.array([860, 300]), np.array([16, 200]))

    v_shelf_21 = Rectangle(np.array([780, 530]), np.array([16, 200]))
    v_shelf_22 = Rectangle(np.array([820, 530]), np.array([16, 200]))


    v_shelf = (v_shelf_1, v_shelf_2, v_shelf_3, v_shelf_4, v_shelf_5, v_shelf_6, v_shelf_7, v_shelf_8, v_shelf_9, v_shelf_10, 
    v_shelf_11, v_shelf_12, v_shelf_13, v_shelf_14, v_shelf_15, v_shelf_16, v_shelf_17, v_shelf_18, v_shelf_19, v_shelf_20,
    v_shelf_21, v_shelf_22)



    p_pillar_1 = Circle(np.array([320, 260]), 10)
    p_pillar_2 = Circle(np.array([420, 260]), 10)
    p_pillar_3 = Circle(np.array([520, 260]), 10)

    p_pillar_4 = Circle(np.array([50, 550]), 10)
    p_pillar_5 = Circle(np.array([50, 650]), 10)
    p_pillar_6 = Circle(np.array([50, 750]), 10)

    p_pillar_7 = Circle(np.array([880, 50]), 10)
    p_pillar_8 = Circle(np.array([880, 150]), 10)
    p_pillar_9 = Circle(np.array([880, 250]), 10)

    p_pillar_10 = Circle(np.array([520, 550]), 10)
    p_pillar_11 = Circle(np.array([620, 550]), 10)
    p_pillar_12 = Circle(np.array([720, 550]), 10)

    p_pillar_13 = Circle(np.array([880, 550]), 10)
    p_pillar_14 = Circle(np.array([880, 650]), 10)
    p_pillar_15 = Circle(np.array([880, 750]), 10)

    p_pillar = (p_pillar_1, p_pillar_2, p_pillar_3, p_pillar_4, p_pillar_5, p_pillar_6, p_pillar_7, p_pillar_8, p_pillar_9, p_pillar_10,
    p_pillar_11, p_pillar_12, p_pillar_13, p_pillar_14, p_pillar_15)



    h_shelf_1 = Rectangle(np.array([250, 50]), np.array([200, 16]))
    h_shelf_2 = Rectangle(np.array([250, 90]), np.array([200, 16]))
    h_shelf_3 = Rectangle(np.array([250, 130]), np.array([200, 16]))
    h_shelf_4 = Rectangle(np.array([250, 170]), np.array([200, 16]))
    h_shelf_5 = Rectangle(np.array([250, 210]), np.array([200, 16]))

    h_shelf_6 = Rectangle(np.array([30, 300]), np.array([240, 16]))
    h_shelf_7 = Rectangle(np.array([30, 340]), np.array([240, 16]))
    h_shelf_8 = Rectangle(np.array([30, 380]), np.array([240, 16]))
    h_shelf_9 = Rectangle(np.array([30, 420]), np.array([240, 16]))
    h_shelf_10 = Rectangle(np.array([30, 460]), np.array([240, 16]))
    #h_shelf_11 = Rectangle(np.array([30, 480]), np.array([240, 16]))

    h_shelf_12 = Rectangle(np.array([620, 50]), np.array([200, 16]))
    h_shelf_13 = Rectangle(np.array([620, 90]), np.array([200, 16]))
    h_shelf_14 = Rectangle(np.array([620, 130]), np.array([200, 16]))
    h_shelf_15 = Rectangle(np.array([620, 170]), np.array([200, 16]))
    h_shelf_16 = Rectangle(np.array([620, 210]), np.array([200, 16]))

    h_shelf_17 = Rectangle(np.array([480, 320]), np.array([200, 16]))
    h_shelf_18 = Rectangle(np.array([480, 360]), np.array([200, 16]))
    h_shelf_19 = Rectangle(np.array([480, 400]), np.array([200, 16]))
    h_shelf_20 = Rectangle(np.array([480, 440]), np.array([200, 16]))
    h_shelf_21 = Rectangle(np.array([480, 480]), np.array([200, 16]))

    h_shelf_22 = Rectangle(np.array([250, 560]), np.array([200, 16]))
    h_shelf_23 = Rectangle(np.array([250, 600]), np.array([200, 16]))
    h_shelf_24 = Rectangle(np.array([250, 640]), np.array([200, 16]))
    h_shelf_25 = Rectangle(np.array([250, 680]), np.array([200, 16]))
    h_shelf_26 = Rectangle(np.array([250, 720]), np.array([200, 16]))

    h_shelf_27 = Rectangle(np.array([500, 600]), np.array([240, 16]))
    h_shelf_28 = Rectangle(np.array([500, 640]), np.array([240, 16]))
    h_shelf_29 = Rectangle(np.array([500, 680]), np.array([240, 16]))
    h_shelf_30 = Rectangle(np.array([500, 720]), np.array([240, 16]))


    h_shelf = (h_shelf_1, h_shelf_2, h_shelf_3, h_shelf_4, h_shelf_5, h_shelf_6, h_shelf_7, h_shelf_8, h_shelf_9, h_shelf_10,
     h_shelf_12, h_shelf_13, h_shelf_14, h_shelf_15, h_shelf_16, h_shelf_17, h_shelf_18, h_shelf_19, h_shelf_20,
     h_shelf_21, h_shelf_22, h_shelf_23, h_shelf_24, h_shelf_25, h_shelf_26, h_shelf_27, h_shelf_28, h_shelf_29, h_shelf_30)

    # Environment variables
    c_space_dimension_boundaries = np.array([1000, 800], dtype='int')
    c_space_obstacle_points = np.array([*v_shelf, *h_shelf])
    c_space_pillar_points = np.array([*p_pillar])

    starting_point = np.array([20, 20])
    car_dimensions = np.array([6, 7.5])

    # Initiate pygame and give permission
    py.init()

    # Create the global rrt planner
    rrt_global_planner = RRT(c_space_dimension_boundaries, c_space_obstacle_points)

    # Create the environment map
    environment_map = Environment(c_space_dimension_boundaries, c_space_obstacle_points, c_space_pillar_points, starting_point, car_dimensions)

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
