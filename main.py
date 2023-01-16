import numpy as np
import pygame as py
import matplotlib.pyplot as plt

from map_environment.environment import Environment
from map_environment.obstacles import Rectangle
from map_environment.obstacles import Circle
from global_planner.rrt import RRT
from local_planner.mpc_controller import MPC
from local_planner.vehicle_dynamics import VehicleDynamics

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
    h_shelf_11 = Rectangle(np.array([30, 480]), np.array([240, 16]))

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

    h_shelf = (
    h_shelf_1, h_shelf_2, h_shelf_3, h_shelf_4, h_shelf_5, h_shelf_6, h_shelf_7, h_shelf_8, h_shelf_9, h_shelf_10,
    h_shelf_12, h_shelf_13, h_shelf_14, h_shelf_15, h_shelf_16, h_shelf_17, h_shelf_18, h_shelf_19, h_shelf_20,
    h_shelf_21, h_shelf_22, h_shelf_23, h_shelf_24, h_shelf_25, h_shelf_26, h_shelf_27, h_shelf_28, h_shelf_29,
    h_shelf_30)

    v_shelf = (
    v_shelf_1, v_shelf_2, v_shelf_3, v_shelf_4, v_shelf_5, v_shelf_6, v_shelf_7, v_shelf_8, v_shelf_9, v_shelf_10,
    v_shelf_11, v_shelf_12, v_shelf_13, v_shelf_14, v_shelf_15, v_shelf_16, v_shelf_17, v_shelf_18, v_shelf_19,
    v_shelf_20, v_shelf_21, v_shelf_22)

    p_pillar = (
    p_pillar_1, p_pillar_2, p_pillar_3, p_pillar_4, p_pillar_5, p_pillar_6, p_pillar_7, p_pillar_8, p_pillar_9,
    p_pillar_10, p_pillar_11, p_pillar_12, p_pillar_13, p_pillar_14, p_pillar_15)

    # Environment variables
    c_space_dimension_boundaries = np.array([550, 550], dtype='int')
    c_space_obstacle_points = np.array([*v_shelf, *h_shelf])
    c_space_pillar_points = np.array([])

    starting_point = np.array([0, 0])
    car_dimensions = np.array([5, 7.5])

    # Controller variables
    max_iterations = 3
    mpc_horizon = 2
    delta_time = 0.1
    current_time = 0.

    control_acceleration = np.array([0.] * mpc_horizon)
    control_delta = np.array([0.] * mpc_horizon)
    previous_closest_reference_point_idx = 1
    shortest_path_coordinates = None

    # State history
    x_pos_history = []
    y_pos_history = []
    yaw_history = []
    vel_history = []
    time_stamps = []

    # Create the local planner
    controller = MPC()
    vehicle_dynamics = VehicleDynamics(sampling_time=delta_time)

    # Initiate pygame and give permission
    py.init()

    # Create the global rrt planner
    rrt_global_planner = RRT(c_space_dimension_boundaries, c_space_obstacle_points, c_space_pillar_points)

    # Create the environment map
    environment_map = Environment(c_space_dimension_boundaries, c_space_obstacle_points, c_space_pillar_points, car_dimensions)

    # Main loop
    running = True
    use_debug = False

    while running:
        py.display.update()

        # Look at every event in the queue
        for event in py.event.get():

            if event.type == py.MOUSEBUTTONUP:
                environment_map.draw_margins = use_debug
                ending_point = np.array(py.mouse.get_pos())

                if not rrt_global_planner.is_node_in_obstacle_space(ending_point):
                    print(f'LOGGER: Valid endpoint given {ending_point}, start building roadmap...')
                    with environment_map.in_progress():
                        rrt_nodes, rrt_edges = rrt_global_planner.build_roadmap(starting_point, ending_point, rewire=True, intermediate_goal_check=True)
                        print(f'LOGGER: Done building roadmap, start calculating shortest path...')

                        shortest_path_edges = rrt_global_planner.generate_shortest_path(starting_point, ending_point)
                        print(f'LOGGER: Done building shortest path, start progression towards goal...')

                        shortest_path_coordinates = rrt_global_planner.coordinates_from_shortest_path(rrt_nodes,shortest_path_edges)
                        reference_x = controller.reference_states(vehicle_dynamics, shortest_path_coordinates)

                    # draw the resulting graph and shortest path nodes and edges
                    if use_debug:
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
            current_yaw, current_vel = int(vehicle_dynamics.yaw), vehicle_dynamics.vel

            time_stamps.append(current_time)
            x_pos_history.append(vehicle_dynamics.x_pos)
            y_pos_history.append(vehicle_dynamics.y_pos)
            yaw_history.append(np.round(vehicle_dynamics.yaw, 2))
            vel_history.append(vehicle_dynamics.vel)

            initial_x = vehicle_dynamics.get_current_state()
            control_vector = None

            closest_reference_point_idx = controller.get_closest_point_index(reference_x, initial_x)
            if closest_reference_point_idx < previous_closest_reference_point_idx:
                closest_reference_point_idx = previous_closest_reference_point_idx

            if reference_x.shape[1] - closest_reference_point_idx == mpc_horizon:
                mpc_horizon = reference_x.shape[1] - closest_reference_point_idx - 1

            if 0 < mpc_horizon:
                next_references = reference_x[:, closest_reference_point_idx:closest_reference_point_idx + mpc_horizon + 1]
                for i in range(max_iterations):
                    prediction_x = vehicle_dynamics.prediction_motion(control_acceleration, control_delta, next_references, mpc_horizon)

                    x_mpc, y_mpc, vel_mpc, phi_mpc, control_acceleration, control_delta = \
                        controller.update_control(vehicle_dynamics, initial_x, next_references, prediction_x, delta_time, mpc_horizon)
                    poa, pod = control_acceleration[:], control_delta[:]
                    du = sum(abs(control_acceleration - poa)) + sum(abs(control_delta - pod))

                    if du <= 0.1:
                        control_vector = np.array([control_acceleration[0], control_delta[0]])
                        vehicle_dynamics.update_state(control_vector)
                        break

                previous_closest_reference_point_idx = closest_reference_point_idx
                py.draw.circle(environment_map.screen, environment_map.RGB_BLUE_CODE, [vehicle_dynamics.x_pos, vehicle_dynamics.y_pos], 3)

                current_time += delta_time
            else:
                # endpoint is reached
                py.time.wait(5000)

                shortest_path_coordinates = None
                vehicle_dynamics.reset_state()

                break


    # stop the pygame simulation
    py.quit()

    if use_debug:
        pos_history = np.array([x_pos_history, y_pos_history])
        ref_history = reference_x[0:2, :].T

        new_y_pos_history = []
        new_x_pos_history = []
        new_yaw_history = []
        new_vel_history = []
        new_time_stamps = []

        for ref in ref_history:
            distances = np.array([np.linalg.norm(ref - pos) for pos in pos_history.T])
            index_closest_value = np.argmin(distances)

            new_y_pos_history.append(y_pos_history[index_closest_value])
            new_x_pos_history.append(x_pos_history[index_closest_value])
            new_yaw_history.append(yaw_history[index_closest_value])
            new_vel_history.append(vel_history[index_closest_value])
            new_time_stamps.append(time_stamps[index_closest_value])

        fig, axs = plt.subplots(2, 2)
        fig.tight_layout()

        axs[0, 0].set_title('X pos')
        axs[0, 0].plot(new_time_stamps, new_x_pos_history, label='History')
        axs[0, 0].plot(new_time_stamps, reference_x[0,:], label='Reference')
        axs[0, 0].legend(loc="upper left")
        axs[0, 0].set_xlabel('Timestamps [sec]')
        axs[0, 0].set_ylabel('Position [dm]')

        axs[0, 1].set_title('Y pos')
        axs[0, 1].plot(new_time_stamps, new_y_pos_history, label='History')
        axs[0, 1].plot(new_time_stamps, reference_x[1,:], label='Reference')
        axs[0, 1].legend(loc="upper left")
        axs[0, 1].set_xlabel('Timestamps [sec]')
        axs[0, 1].set_ylabel('Position [dm]')

        axs[1, 0].set_title('Vehicle velocity')
        axs[1, 0].plot(new_time_stamps, new_vel_history, label='History')
        axs[1, 0].plot(new_time_stamps, reference_x[2, :], label='Reference')
        axs[1, 0].legend(loc="upper left")
        axs[1, 0].set_xlabel('Timestamps [sec]')
        axs[1, 0].set_ylabel('Velocity [dm/s]')

        axs[1, 1].set_title('Vehicle yaw')
        axs[1, 1].plot(new_time_stamps, new_yaw_history, label='History')
        axs[1, 1].plot(new_time_stamps, reference_x[3, :], label='Reference')
        axs[1, 1].legend(loc="upper left")
        axs[1, 1].set_xlabel('Timestamps [sec]')
        axs[1, 1].set_ylabel('Angle [rad]')

        plt.show()
