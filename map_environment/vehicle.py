import numpy as np

class BicycleModel:
    def __init__(self, m, I, a, L):
        self.m = m  # mass of the vehicle
        self.I = I  # vehicle moment of inertia around Center of Mass(CoM)
        self.a = a  # distance rear axle to CoM
        self.L = L  # wheel base (distance between front and rear axle)
        self.b = L - a  # distance front axle to CoM
        self.q = [0, 0, 0]  # x- and y-position and direction of vehicle
        self.dq = [0.0, 0.0, 0.0]  # x- and y-veloctiy and rotational velocity of vehicle

    def state(self, q, dq, delta):
        # delta: angle of the front tires
        phi = q[2]  # angle of the vehicle

        # constrain matrix??
        c = np.array([[np.sin(phi), -np.cos(phi), self.a],
                      [np.sin(phi + delta), -np.cos(phi + delta), -self.b * np.cos(delta)]])

        jacobian = np.linalg.pinv(c)
        jac_c = np.dot(jacobian, c)
        update_dq = -np.dot(jac_c, dq)

        self.q = q
        self.dq = dq + update_dq

    def forward_Dynamics(self, delta, Fa):
        cosphi = np.cos(self.q[2])
        sinphi = np.sin(self.q[2])
        cosdeltaphi = np.cos(self.q[2] + delta)
        sindeltaphi = np.sin(self.q[2] + delta)
        cosdelta = np.cos(delta)
        dx = self.dq[0]
        dy = self.dq[1]
        dphi = self.dq[2]
        a = self.a
        b = self.b

        # Force from ground on rear axle
        # Inertia Matrix
        M = np.array([[self.m, 0, 0],
                      [0, self.m, 0],
                      [0, 0, self.I]])

        rhs = np.array([Fa * cosphi, Fa * sinphi, 0])

        sol = np.linalg.solve(M, rhs)
        ddq = sol[0:3]

        return ddq
