import math
import cvxpy as cp
import numpy as np


class MPC:
    def __init__(self):
        """Initialize the variables of the mpc_controller
        :param x_vec_len: the number of states of the system
        :param u_vec_len: the number of control inputs of the system
        :param state_weights: the weight matrix on the states
        :param input_weights: the weight matrix on the control inputs
        :param input_difference_weights: the weight matrix on the difference between the control input and the previous control input
        """
        self.x_vec_len = 4
        self.u_vec_len = 2

        # the input and control weights
        self.state_weights = np.diag([12., 10., 1., 0.1])
        self.input_weights = np.diag([0.1, 0.1])
        self.input_difference_weights = np.diag([0.1, 0.5]) 

    def update_control(self, vehicle, initial_state, reference_x, prediction_x, sampling_time, prediction_horizon):
        """Define the optimzation problem of the MPC controller and solve the optimaztion problem using cvx
        :param vehicle: contains the vehicle dynamics variables
        :param initial_state:the initial state of the mobile robot
        :param referece_x: the current array of reference states for the mpc controller
        :param prediction_x: the predicted states over the prediction horizon
        :param sampling_time: the sampling time of the discrete time system
        :param prediction_horizon: the prediction horizon of the mpc_controller
        
        :return:
            The states and control inputs from the MPC controller after solving the optimization problem
        """
        #initialize the optimization variables, the cost and constraints
        x = cp.Variable((self.x_vec_len, prediction_horizon + 1))
        u = cp.Variable((self.u_vec_len, prediction_horizon))

        cost = 0.0
        constraints = []

        # start the MPC loop
        for t in range(prediction_horizon):
            #cost of the input
            cost += cp.quad_form(u[:, t], self.input_weights)

            if t != 0:
                #cost of the error between the current state and the reference
                cost += cp.quad_form(reference_x[:, t] - x[:, t], self.state_weights)
            
            #the initial codition constraint    
            constraints += [x[:, 0] == initial_state]
            
            #calculate the linearized state space matrices for the current state
            mat_a, mat_b, mat_c = vehicle.linearized_model(prediction_x[2, t], prediction_x[3, t], 0.0)
            constraints += [x[:, t + 1] == mat_a @ x[:, t] + mat_b @ u[:, t] + mat_c] #add the dynamic constraints

            if t < (prediction_horizon - 1):
                #the cost of the change in control input
                cost += cp.quad_form(u[:, t + 1] - u[:, t], self.input_difference_weights)
                #the constraint on the change in control input
                constraints += [cp.abs(u[1, t + 1] - u[1, t]) <= vehicle.MAX_STEER_ANGLE_SPEED * sampling_time]

        # constant state and input constraints
        constraints += [x[2, :] <= vehicle.MAX_SPEED]
        constraints += [x[2, :] >= vehicle.MIN_SPEED]
        constraints += [cp.abs(u[1, :]) <= vehicle.MAX_STEER_ANGLE]
        constraints += [cp.abs(u[0, :]) <= vehicle.MAX_ACCEL]

        cost += cp.quad_form(reference_x[:, prediction_horizon] - x[:, prediction_horizon], self.state_weights)

        # solve the optimization problem
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve(solver=cp.OSQP)

        if problem.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
            print('MPC not solvable')
            raise RuntimeError

        # get the states from the solver
        x_mpc = np.array(x.value[0, :]).flatten()
        y_mpc = np.array(x.value[1, :]).flatten()
        v_mpc = np.array(x.value[2, :]).flatten()
        phi_mpc = np.array(x.value[3, :]).flatten()
        acc_mpc = np.array(u.value[0, :]).flatten()
        delta_mpc = np.array(u.value[1, :]).flatten()

        return x_mpc, y_mpc, v_mpc, phi_mpc, acc_mpc, delta_mpc

    @staticmethod
    def reference_states(vehicle, coordinates):
        """Generate the reference states from the coordinates of the RRT algorithm
        :param vehicle: the class containing the variables of the vehicle dynamics
        :param coordinates: the discretized coordinates from the RRT algorithm
        
        :return:
            An array containing the reference states from start coordinate to end coordinate
        """
        reference_x_pos, reference_y_pos = list(coordinates[:, 0]), list(coordinates[:, 1])
        reference_vel = np.array([vehicle.TARGET_SPEED] * len(reference_x_pos))

        reference_yaw = [0.]
        for pos_idx in range(1, len(reference_x_pos)):
            x_prev, y_prev = reference_x_pos[pos_idx - 1], reference_y_pos[pos_idx - 1]
            x, y = reference_x_pos[pos_idx], reference_y_pos[pos_idx]

            reference_yaw_coordinate = 0.
            if int(x * y)  > 0:
                if x_prev < x:
                    reference_yaw_coordinate = math.atan(y/x)
                if x < x_prev:
                    reference_yaw_coordinate += np.deg2rad(90.0)
                if y < y_prev:
                    reference_yaw_coordinate = -reference_yaw_coordinate

            reference_yaw.append(reference_yaw_coordinate)

        return np.array([reference_x_pos, reference_y_pos, reference_vel, reference_yaw])

    @staticmethod
    def get_closest_point_index(reference_x, current_x):
        """ Find the closest reference location to the current location
        :param referece_x: the current array of reference states for the mpc controller        
        :param current_x: the current states
        """
        current_coordinate = np.array([current_x[0], current_x[1]])
        all_coordinates = reference_x[:2,:].T

        distances = np.array([np.linalg.norm(coordinate - current_coordinate) for coordinate in all_coordinates])
        return np.argmin(distances)