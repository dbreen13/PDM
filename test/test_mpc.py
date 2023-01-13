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
    max_iterations = 3
    mpc_horizon = 5

    target_vel = 5.0
    target_yaw = np.deg2rad(45.0)

    max_run_time = 300.
    current_time = 0.
    delta_time = 0.5

    goal_distance = 1.0  # goal distance
    stop_speed = 1.0 / 3.6  # stop speed
    
    
    positions=np.array([[0,0],[10,10],[10,30],[25,40],[35,30]])
    #x_ref=np.zeros((4,len(positions)*(mpc_horizon+1)))
    x_ref=np.array([0,0,0,0])
    for i in range(len(positions)-1):
        start_pos=np.array(positions[i])
        ending_pos=np.array(positions[i+1])
        path_edge=np.array([start_pos,ending_pos])
        
        discretize_edges = np.transpose(discretize_edge(path_edge, points=20))
        reference_x_pos = discretize_edges[0, :]
        reference_y_pos = discretize_edges[1, :]
        target_yaw=math.atan((reference_y_pos[1]/reference_x_pos[1]))
        reference_vel = np.array([target_vel] * reference_x_pos.shape[0])
        reference_yaw = np.array([target_yaw] * reference_x_pos.shape[0])
        
        reference_x = np.array([reference_x_pos, reference_y_pos, reference_vel, reference_yaw])
        reference_x_2  = np.transpose(reference_x)
        x_ref=np.vstack((x_ref,reference_x_2))
        
    reference_x=np.transpose(x_ref[1:])

    # start_pos = np.array([0, 0])
    # ending_pos = np.array([10, 10])
    # path_edge = np.array([start_pos, ending_pos])

    # discretize_edge = np.transpose(discretize_edge(path_edge, points=mpc_horizon+1))
    # reference_x_pos = discretize_edge[0, :]
    # reference_y_pos = discretize_edge[1, :]
    # reference_vel = np.array([target_vel] * reference_x_pos.shape[0])
    # reference_yaw = np.array([target_yaw] * reference_x_pos.shape[0])

    # reference_x = np.array([reference_x_pos, reference_y_pos, reference_vel, reference_yaw])

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
        current_yaw, current_vel = vehicle_dynamics.yaw, vehicle_dynamics.vel

        x_pos_history.append(current_x_pos)
        y_pos_history.append(current_y_pos)
        yaw_history.append(current_yaw)
        vel_history.append(current_vel)

        time_stamps.append(current_time)
        current_time += delta_time

        initial_x = vehicle_dynamics.get_current_state()

        # if check_goal(initial_x, np.array([35,30]), stop_speed, goal_distance):
        #     print('Goal has been reached.')
        #     break
    
        current_coordinate = np.array([current_x_pos, current_y_pos])
        coordinates = reference_x[:2,:].T
        
        distances = np.array([np.linalg.norm(point - current_coordinate) for point in coordinates])
        closest_point_idx = np.argmin(distances)
        
        if closest_point_idx>=(len(distances)-6):
            break

        next_references = reference_x[:, closest_point_idx:closest_point_idx + mpc_horizon+1]
        
        for i in range(max_iterations):
            prediction_x = vehicle_dynamics.prediction_motion(control_acceleration, control_delta,next_references  , mpc_horizon)
            x_mpc, y_mpc, vel_mpc, phi_mpc, control_acceleration, control_delta = \
                mpc_controller.update_control(vehicle_dynamics, initial_x, next_references, prediction_x, delta_time, mpc_horizon)

            poa, pod = control_acceleration[:], control_delta[:]
            du = sum(abs(control_acceleration - poa)) + sum(abs(control_delta - pod))  # calc u change value

            if du <= 0.1:
                control_vector = np.array([control_acceleration[0], control_delta[0]])
                vehicle_dynamics.update_state(control_vector)
                break



    fig, ax = plt.subplots()
    ax.plot(reference_x[0,:], reference_x[1,:], color='red', linestyle='--', linewidth=1)

    for x_ref_pos, y_ref_pos in zip(reference_x[0,:], reference_x[1,:]):
        ax.scatter(x_ref_pos, y_ref_pos, s=10, c="red")

    for x_pos_mpc, y_pos_mpc in zip(x_pos_history, y_pos_history):
        ax.scatter(x_pos_mpc, y_pos_mpc, s=5, c="blue")

    plt.grid(True)
    plt.show()
