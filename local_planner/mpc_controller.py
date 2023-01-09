import cvxpy as cp
import numpy as np


class MPC:
    def __init__(self, horizon):
        self.prediction_horizon = horizon
        self.x_vec_len = 4
        self.u_vec_len = 2

        # the input and control weights
        self.state_weights = 10.0 * np.eye(self.x_vec_len)
        self.input_weights = 0.01 * np.eye(self.u_vec_len)
        self.input_difference_weights = 0.01 * np.eye(self.u_vec_len)

    def update_control(self, vehicle, initial_state, reference_x, prediction_x, sampling_time):
        """ """
        x = cp.Variable((self.x_vec_len, self.prediction_horizon + 1))
        u = cp.Variable((self.u_vec_len, self.prediction_horizon))

        cost = 0.0
        constraints = []

        # start the MPC loop
        for t in range(self.prediction_horizon):
            cost += cp.quad_form(u[:, t], self.input_weights)

            if t != 0:
                cost += cp.quad_form(reference_x[:, t] - x[:, t], self.state_weights)

            constraints += [x[:, 0] == initial_state]

            mat_a, mat_b, mat_c = vehicle.linearized_model(prediction_x[2, t], prediction_x[3, t], 0.0)
            constraints += [x[:, t + 1] == mat_a @ x[:, t] + mat_b @ u[:, t] + mat_c]

            if t < (self.prediction_horizon - 1):
                cost += cp.quad_form(u[:, t + 1] - u[:, t], self.input_difference_weights)
                constraints += [cp.abs(u[1, t + 1] - u[1, t]) <= vehicle.MAX_STEER_ANGLE * sampling_time]

        # constant state constraints
        constraints += [x[2, :] <= vehicle.MAX_SPEED]
        constraints += [x[2, :] >= vehicle.MIN_SPEED]

        constraints += [cp.abs(u[1, :]) <= vehicle.MAX_STEER_ANGLE]
        constraints += [cp.abs(u[0, :]) <= vehicle.MAX_ACCEL]

        cost += cp.quad_form(reference_x[:, self.prediction_horizon] - x[:, self.prediction_horizon],
                             self.state_weights)

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