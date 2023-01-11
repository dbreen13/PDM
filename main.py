
import numpy as np
import pygame as py

from map_environment.environment import Environment
from map_environment.obstacles import Rectangle
from global_planner.rrt import RRT
from local_planner.mpc_controller import MPC
from local_planner.vehicle_dynamics import VehicleDynamics


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

    starting_point = np.array([0, 0])
    car_dimensions = np.array([5, 7.5])

    # Controller variables
    mpc_horizon = 3
    max_iterations = 3
    delta_time = 0.1

    control_acceleration = np.array([0.] * mpc_horizon)
    control_delta = np.array([0.] * mpc_horizon)
    previous_closest_reference_point_idx = 1
    shortest_path_coordinates = None

    # Create the local planner
    controller = MPC()
    vehicle_dynamics = VehicleDynamics(sampling_time=delta_time)

    # Initiate pygame and give permission
    py.init()

    # Create the global rrt planner
    rrt_global_planner = RRT(c_space_dimension_boundaries, c_space_obstacle_points)

    # Create the environment map
    environment_map = Environment(c_space_dimension_boundaries, c_space_obstacle_points, car_dimensions)

    # Main loop
    running = True
    while running:
        py.display.update()

        # Look at every event in the queue
        for event in py.event.get():

            if event.type == py.MOUSEBUTTONUP:
                ending_point = np.array(py.mouse.get_pos())

                with environment_map.in_progress():
                    rrt_nodes, rrt_edges = rrt_global_planner.build_roadmap(starting_point, ending_point)
                    shortest_path_edges = rrt_global_planner.generate_shortest_path(starting_point, ending_point)

                    shortest_path_coordinates = rrt_global_planner.coordinates_from_shortest_path(rrt_nodes, shortest_path_edges)
                    reference_x = controller.reference_states(vehicle_dynamics, shortest_path_coordinates)

                # draw the resulting graph and shortest path nodes and edges
                environment_map.draw_path(rrt_nodes, rrt_edges, shortest_path_edges)

                # draw coordinates
                environment_map.draw_coordinates(shortest_path_coordinates)

            if event.type == py.KEYDOWN:
                # Was it the Escape key? If so, stop the loop.
                if event.key == py.K_ESCAPE:
                    running = False

            # Did the user click the window close button? If so, stop the loop.
            elif event.type == py.QUIT:
                running = False

        if shortest_path_coordinates is not None:
            current_x_pos, current_y_pos = vehicle_dynamics.x_pos, vehicle_dynamics.y_pos
            current_yaw, current_vel = vehicle_dynamics.yaw, vehicle_dynamics.vel

            initial_x = vehicle_dynamics.get_current_state()
            control_vector = None

            closest_reference_point_idx = controller.get_closest_point_index(reference_x, initial_x)
            if closest_reference_point_idx < previous_closest_reference_point_idx:
                closest_reference_point_idx = previous_closest_reference_point_idx

            if reference_x.shape[1] - closest_reference_point_idx == mpc_horizon:
                mpc_horizon = reference_x.shape[1] - closest_reference_point_idx - 1

            if mpc_horizon == 0:
                # shortest_path_coordinates = None
                vehicle_dynamics.reset_state()
                break

            next_references = reference_x[:, closest_reference_point_idx:closest_reference_point_idx + mpc_horizon + 1]
            for i in range(max_iterations):
                prediction_x = vehicle_dynamics.prediction_motion(control_acceleration, control_delta, next_references, mpc_horizon)

                x_mpc, y_mpc, vel_mpc, phi_mpc, control_acceleration, control_delta = \
                    controller.update_control(vehicle_dynamics, initial_x, next_references, prediction_x, delta_time, mpc_horizon)

                poa, pod = control_acceleration[:], control_delta[:]
                du = sum(abs(control_acceleration - poa)) + sum(abs(control_delta - pod))  # calc u change value

                if du <= 0.1:
                    control_vector = np.array([control_acceleration[0], control_delta[0]])
                    vehicle_dynamics.update_state(control_vector)
                    break

            previous_closest_reference_point_idx = closest_reference_point_idx

            py.draw.circle(environment_map.screen, environment_map.RGB_BLUE_CODE, [vehicle_dynamics.x_pos, vehicle_dynamics.y_pos], 3)
