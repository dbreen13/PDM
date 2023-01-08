import math

import numpy as np
import matplotlib.pyplot as plt

from local_planner.vehicle_dynamics import VehicleDynamics
from local_planner.mpc_controller import MPC

def discretize_edge(edge, points=50):
    """ Linearly discretize an edge into multiple points.
    :param points:
        Amount of points used in the discretize process.
    :param edge:
        The edge used in the discretize process.
    :return:
        Set of intermediate nodes.
    """
    start_point, end_point = edge[0], edge[1]
    intermediate_nodes = start_point + np.linspace(0, 1, points)[..., np.newaxis] * (end_point - start_point)

    return intermediate_nodes

def check_goal(state, goal_position, max_stop_speed, min_goal_distance):
    """ """
    dx = state[0] - goal_position[0]
    dy = state[1] - goal_position[1]
    d = math.hypot(dx, dy)

    is_goal = (d <= min_goal_distance)
    is_stop = (abs(state[2]) <= max_stop_speed)

    if is_goal and is_stop:
        return True

    return False

if __name__ == '__main__':
    max_iterations = 5
    mpc_horizon = 3

    target_vel = 2.0
    target_yaw = np.deg2rad(45.0)

    max_run_time = 25.
    current_time = 0.
    delta_time = 0.2

    goal_distance = 1.0  # goal distance
    stop_speed = 1.0 / 3.6  # stop speed

    start_pos = np.array([0, 0])
    ending_pos = np.array([10, 5])
    path_edge = np.array([start_pos, ending_pos])

    discretize_edge = np.transpose(discretize_edge(path_edge, points=mpc_horizon+1))
    reference_x_pos = discretize_edge[0, :]
    reference_y_pos = discretize_edge[1, :]
    reference_vel = np.array([target_vel] * reference_x_pos.shape[0])
    reference_yaw = np.array([target_yaw] * reference_x_pos.shape[0])

    reference_x = np.array([reference_x_pos, reference_y_pos, reference_vel, reference_yaw])

    vehicle_dynamics = VehicleDynamics()
    mpc_controller = MPC(mpc_horizon)

    x_pos_history = []
    y_pos_history = []
    yaw_history = []
    vel_history = []
    time_stamps = []

    control_acceleration = np.array([0.] * mpc_horizon)
    control_delta = np.array([0.] * mpc_horizon)

    while current_time < max_run_time:
        current_x_pos, current_y_pos = vehicle_dynamics.x_pos, vehicle_dynamics.y_pos
        current_yaw, current_vel = vehicle_dynamics.yaw, vehicle_dynamics.velocity

        x_pos_history.append(current_x_pos)
        y_pos_history.append(current_y_pos)
        yaw_history.append(current_yaw)
        vel_history.append(current_vel)

        time_stamps.append(current_time)
        current_time += delta_time

        initial_x = vehicle_dynamics.get_current_state()

        if check_goal(initial_x, ending_pos, stop_speed, goal_distance):
            print('Goal has been reached.')
            break

        for i in range(max_iterations):
            prediction_x = vehicle_dynamics.prediction_motion(control_acceleration, control_delta, reference_x, mpc_horizon)
            x_mpc, y_mpc, vel_mpc, phi_mpc, control_acceleration, control_delta = \
                mpc_controller.update_control(vehicle_dynamics, initial_x, reference_x, prediction_x, delta_time)

            poa, pod = control_acceleration[:], control_delta[:]
            du = sum(abs(control_acceleration - poa)) + sum(abs(control_delta - pod))  # calc u change value

            if du <= 0.1:
                control_vector = np.array([control_acceleration[0], control_delta[0]])
                vehicle_dynamics.update_state(control_vector)
                break



    fig, ax = plt.subplots()
    ax.plot(reference_x_pos, reference_y_pos, color='red', linestyle='--', linewidth=1)

    for x_ref_pos, y_ref_pos in zip(reference_x_pos, reference_y_pos):
        ax.scatter(x_ref_pos, y_ref_pos, s=10, c="red")

    for x_pos_mpc, y_pos_mpc in zip(x_pos_history, y_pos_history):
        ax.scatter(x_pos_mpc, y_pos_mpc, s=5, c="blue")

    plt.grid(True)
    plt.show()
