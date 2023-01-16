import numpy as np
import control as ctrl


class VehicleDynamics:

    SAMPLING_TIME = 0.10
    WHEEL_BASE_LENGTH = 7.5  # length of the wheelbase [dm]

    TARGET_SPEED = 13.0  # target speed [dm/s]

    MAX_STEER_ANGLE = np.deg2rad(45.0)  # maximum steering angle [rad]
    MAX_STEER_ANGLE_SPEED = np.deg2rad(30.0)  # maximum steering angle [rad/s]

    MAX_SPEED = 15.  # maximum speed [dm/s]
    MIN_SPEED = 0.0  # minimum speed [dm/s]
    MAX_ACCEL = 0.75  # maximum accel [dm/ss]

    def __init__(self, x_pos=0.0, y_pos=0.0, yaw=0.0, velocity=0.0, sampling_time=SAMPLING_TIME):
        """ Create a variable object which can be used for the vehicle dynamics."""
        self.dt = sampling_time # The sampling time

        self.x_pos = x_pos # The position in the x-direction
        self.y_pos = y_pos # The position in the y-direction
        self.vel = velocity # The longitudinal velocity
        self.yaw = yaw # The yaw of the mobile robot

        self.A = np.eye(4) # Initialization of the A matrix
        self.B = np.zeros((4, 2)) # Initialization of the B matrix
        self.C = np.zeros(4) # Initialization of the C matrix

    def reset_state(self):
        """ Reset the state to the initial conditions."""
        self.x_pos = 0.
        self.y_pos = 0.
        self.vel = 0.
        self.yaw = 0.

    def get_current_state(self):
        """Return the values of the current state as an np.array."""
        return np.array([self.x_pos, self.y_pos, self.vel, self.yaw])

    def update_state(self, control_vec):
        """Update the current state using the nonlinear dynamics and the control input
        :param control_vec: the current control input vector
        """
        acceleration, steering_angle_delta = control_vec

        self.x_pos += (self.vel * np.cos(self.yaw)) * self.dt
        self.y_pos += (self.vel * np.sin(self.yaw)) * self.dt
        self.vel += acceleration * self.dt

        new_yaw = self.yaw + (self.vel / self.WHEEL_BASE_LENGTH * np.tan(steering_angle_delta) * self.dt)
        self.yaw = self.optimize_angle(self.yaw, new_yaw)

    def prediction_motion(self, ctrl_acc, ctrl_delta, reference_x, mpc_horizon):
        """Predict the next states over the prediciton horizon
        :param ctrl_acc: the control input value of the acceleration
        :param ctrl_delta: the control input value of the steering wheel angle delta
        :param mpc_horizon: the prediction horizon of the mpc controller
        :param reference_x: the reference state 
        
        :return: x_prediction
            An array with the predicted states over the mpc_horizon
        """
        x_prediction = np.zeros(reference_x.shape)

        current_x_pos, current_y_pos = self.x_pos, self.y_pos
        current_yaw, current_vel = self.yaw, self.vel

        x_prediction[0, 0] = current_x_pos
        x_prediction[1, 0] = current_y_pos
        x_prediction[2, 0] = current_vel
        x_prediction[3, 0] = current_yaw

        for (ctrl_acc_val, ctrl_delta_val, idx) in zip(ctrl_acc, ctrl_delta, range(1, mpc_horizon + 1)):
            u_control_input = np.array([ctrl_acc_val, ctrl_delta_val])
            self.update_state(u_control_input)

            x_prediction[0, idx] = self.x_pos
            x_prediction[1, idx] = self.y_pos
            x_prediction[2, idx] = self.vel
            x_prediction[3, idx] = self.yaw

        self.x_pos, self.y_pos = current_x_pos, current_y_pos
        self.yaw, self.vel = current_yaw, current_vel

        return x_prediction

    def linearized_model(self, velocity, yaw, steering_angle):
        """Linearize the bicycle model around the current state and check controllability every timestep
        :param velocity: the current velocity of the mobile robot
        :param yaw: the current yaw of the mobile robot
        :param steering_angle: the current steering wheel angle of the mobile robot
        
        :return:
            The state space matrices A,B,C of the linearized system
        """
        self.A[0, 2] = self.dt * np.cos(yaw)
        self.A[0, 3] = -self.dt * velocity * np.sin(yaw)
        self.A[1, 2] = self.dt * np.sin(yaw)
        self.A[1, 3] = self.dt * velocity * np.cos(yaw)
        self.A[3, 2] = self.dt * np.tan(steering_angle) / self.WHEEL_BASE_LENGTH

        self.B[2, 0] = self.dt
        self.B[3, 1] = self.dt * velocity / (self.WHEEL_BASE_LENGTH * np.cos(steering_angle) ** 2)

        self.C[0] = self.dt * velocity * np.sin(yaw) * yaw
        self.C[1] = - self.dt * velocity * np.cos(yaw) * yaw
        self.C[3] = - self.dt * velocity * steering_angle / (self.WHEEL_BASE_LENGTH * np.cos(steering_angle) ** 2)

        # check full rank controllability
        ctrl_mat = ctrl.ctrb(self.A, self.B)
        4 - np.linalg.matrix_rank(ctrl_mat)

        return self.A, self.B, self.C

    @staticmethod
    def optimize_angle(previous_angle, new_angle):
        """ Smoothen the angle to [-pi/2, pi/2].
        :param previous_angle:
            The previous angle of the vehicle
        :param new_angle:
            The new angle of the vehicle

        :returns:
            new angle but smooth, which means it is restricted to [-pi/2, pi/2]
        """
        smooth_angle = new_angle

        angle_diff = new_angle - previous_angle
        while angle_diff >= np.pi / 2.0:
            smooth_angle -= 2.0 * np.pi

        while angle_diff <= -np.pi / 2.0:
            smooth_angle += 2.0 * np.pi

        return smooth_angle
