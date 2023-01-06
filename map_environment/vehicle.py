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



'SIMULATION'

import pygame
import math

# Set up Pygame
pygame.init()

# Set up the window
width, height = 550, 550
screen = pygame.display.set_mode((width, height))

# Draw the path on the screen
path_color = (0, 0, 255)
path_width = 3
pygame.draw.line(screen, path_color, (0, 0), ([100, 100]), path_width)


# Define the Car class
class Car:
    def __init__(self, x, y, v, phi, a, delta):
        self.x = x # x-position of the vehicle
        self.y = y # y-position of the vehicle
        self.v = v # velocity of the vehicle
        self.phi = phi # orientation angle of the vehicle
        self.acc = a # acceleration of the vehicle
        self.delta = delta # orientation angle of the front wheels of the vehicle

        self.m = 0.5
        self.L = 1
        self.I = (1/12)*self.m*self.L**2
        self.a = self.L/2
        self.b = self.L - self.a



    def update(self):
        # Update the position and velocity of the car using the equations of motion
        self.x += self.v * math.cos(self.phi)
        self.y += self.v * math.sin(self.phi)
        self.v += self.acc
        self.phi += self.v * math.tan(self.delta) / self.L

        
# Create an instance of the Car class
car = Car(0, 0, 0, 0, 0, 0)
#car = Car(x_mpc,y_mpc,v_mpc,phi_mpc,a_mpc,delta_mpc) # The state-vector values that come from the MPC


# Main loop
while True:
    # Update the car's position and velocity
    car.update()

    # Draw the car on the screen
    car_color = (255, 0, 0)
    car_size = (20, 30)
    car_rect = pygame.Rect(car.x, car.y, car_size[0], car_size[1])
    pygame.draw.rect(screen, car_color, car_rect)

    # Update the Pygame display
    pygame.display.flip()

