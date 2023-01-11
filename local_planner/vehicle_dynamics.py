import numpy as np


class VehicleDynamics:
    SAMPLING_TIME = 0.10
    WHEEL_BASE_LENGTH = 7.5

    TARGET_SPEED = 10.0

    MAX_STEER_ANGLE = np.deg2rad(45.0)  # maximum steering angle [rad]
    MAX_SPEED = 11.  # maximum speed [dm/s]
    MIN_SPEED = 0.0  # minimum speed [dm/s]
    MAX_ACCEL = 0.75  # maximum accel [dm/ss]

    def __init__(self, x_pos=0.0, y_pos=0.0, yaw=0.0, velocity=0.0, sampling_time=SAMPLING_TIME):
        self.dt = sampling_time

        self.x_pos = x_pos
        self.y_pos = y_pos
        self.vel = velocity
        self.yaw = yaw

        self.A = np.eye(4)
        self.B = np.zeros((4, 2))
        self.C = np.zeros(4)

    def reset_state(self):
        """ """
        self.x_pos = 0.
        self.y_pos = 0.
        self.vel = 0.
        self.yaw = 0.

    def get_current_state(self):
        return np.array([self.x_pos, self.y_pos, self.vel, self.yaw])

    def update_state(self, control_vec):
        """ """
        acceleration, steering_angle_delta = control_vec

        self.x_pos += (self.vel * np.cos(self.yaw)) * self.dt
        self.y_pos += (self.vel * np.sin(self.yaw)) * self.dt
        self.vel += acceleration * self.dt

        self.yaw += self.vel / self.WHEEL_BASE_LENGTH * np.tan(steering_angle_delta) * self.dt
        self.yaw = self.normalize_angle(self.yaw)

    def prediction_motion(self, ctrl_acc, ctrl_delta, reference_x, mpc_horizon):
        """ """
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
        """ """
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

        return self.A, self.B, self.C

    @staticmethod
    def normalize_angle(angle):
        """ Normalize an angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle
