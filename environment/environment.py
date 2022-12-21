
import numpy as np
import pygame

class Environment:

    RGB_WHITE_CODE = (255, 255, 255)
    RGB_BLUE_CODE = (0, 0, 255)

    def __init__(self, c_space_dimensions: np.array, c_space_obstacles: np.array):
        self.c_space_dimensions = c_space_dimensions
        self.c_space_obstacles = c_space_obstacles

        self.screen = pygame.display.set_mode(c_space_dimensions)
        self.screen.fill(self.RGB_WHITE_CODE)

