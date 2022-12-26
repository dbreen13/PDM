
import numpy as np
import pygame
from shapely.geometry.polygon import Polygon

class Environment:

    RGB_WHITE_CODE = (255, 255, 255)
    RGB_BLUE_CODE = (0, 0, 255)
    RGB_RED_CODE = (255, 0, 0)

    def __init__(self, c_space_dimensions: np.array, c_space_obstacles: np.array):
        self.c_space_dimensions = c_space_dimensions
        self.c_space_obstacles = c_space_obstacles

        self.screen = pygame.display.set_mode(c_space_dimensions)
        self.screen.fill(self.RGB_WHITE_CODE)

        
        pygame.draw.rect(self.screen, self.RGB_BLUE_CODE, pygame.Rect(30, 30, 200, 20))
        pygame.draw.rect(self.screen, self.RGB_BLUE_CODE, pygame.Rect(270, 30, 200, 20))

        for x in [30, 90, 150, 210, 270, 330, 390, 450]:
            pygame.draw.rect(self.screen, self.RGB_BLUE_CODE, pygame.Rect(x, 90, 20, 300))

        pygame.draw.rect(self.screen, self.RGB_BLUE_CODE, pygame.Rect(30, 430, 200, 20))
        pygame.draw.rect(self.screen, self.RGB_BLUE_CODE, pygame.Rect(270, 430, 200, 20))

        
        obs_x, obs_y = (70, 70)
        vertices = [(obs_x-10, obs_y-10), (obs_x-15, obs_y), (obs_x-10, obs_y+10), (obs_x+5, obs_y+10), (obs_x+10, obs_y), (obs_x+5, obs_y-10)]
        pygame.draw.polygon(self.screen, self.RGB_RED_CODE, vertices)
        
        pygame.display.flip()
    
class bicycle_model:
    def __init__(self, m, I, a, L):
        self.m = m # mass of the vehicle 
        self.I = I # vehicle moment of inertia around Center of Mass(CoM)
        self.a = a # distance rear axle to CoM
        self.L = L # wheel base (distance between front and rear axle)
        self.b = L-a  # distance front axle to CoM
        self.q = [0, 0, 0] # x- and y-position and direction of vehicle
        self.dq = [0.0, 0.0, 0.0] # x- and y-veloctiy and rotational velocity of vehicle
    
    def state(self, q, dq, delta):
        #delta: angle of the front tires
        phi = q[2] #angle of the vehicle
        
        #constrain matrix??
        C = np.array([[    np.sin(phi),         -np.cos(phi),            self.a],
                      [np.sin(phi+delta), -np.cos(phi+delta), -self.b*np.cos(delta)]])
        
        
        Jacobian = np.linalg.pinv(C)
        Jac_C = np.dot(Jacobian,C)
        update_dq = -np.dot(Jac_C,dq)
        self.q = q
        self.dq = dq + update_dq

    
    def Forward_Dynamics(self,delta, Fa):
        cosphi = np.cos(self.q[2])
        sinphi = np.sin(self.q[2])
        cosdeltaphi = np.cos(self.q[2]+delta)
        sindeltaphi = np.sin(self.q[2]+delta)
        cosdelta    = np.cos(delta)
        dx   = self.dq[0]
        dy   = self.dq[1]
        dphi = self.dq[2]
        a = self.a
        b = self.b

        # Force from ground on rear axle
        # Inertia Matrix
        M = np.array([[self.m, 0, 0],
                      [0, self.m, 0],
                      [0, 0, self.I]])
        
        rhs = np.array([Fa*cosphi, Fa*sinphi, 0])

        sol = np.linalg.solve(M, rhs)       
        ddq = sol[0:3]

        return ddq



