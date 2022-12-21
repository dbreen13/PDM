
import numpy as np
import pygame

from environment.environment import Environment
from global_planner.rrt import RRT


if __name__ == '__main__':
    # Program variables
    c_space_dimension_boundaries = np.array([500, 500], dtype='int')
    
    obs_x, obs_y = (70, 70)
    c_space_obstacle_points = np.array([[(obs_x-10, obs_y-10), (obs_x-15, obs_y), (obs_x-10, obs_y+10), (obs_x+5, obs_y+10), (obs_x+10, obs_y), (obs_x+5, obs_y-10)]])

    # Initiate pygame and give permission
    pygame.init()

    # Create the global rrt planner
    rrt = RRT(c_space_dimension_boundaries, c_space_obstacle_points)

    # Create the environment map
    environment_map = Environment(c_space_dimension_boundaries, c_space_obstacle_points)

    # Main loop
    running = True
    while running:
        pygame.display.update()

        # Look at every event in the queue
        for event in pygame.event.get():
            # Did the user hit a key?
            if event.type == pygame.KEYDOWN:
                # Was it the Escape key? If so, stop the loop.
                if event.key == pygame.K_ESCAPE:
                    running = False

            # Did the user click the window close button? If so, stop the loop.
            elif event.type == pygame.QUIT:
                running = False
