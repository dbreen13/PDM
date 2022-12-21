
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

