
import numpy as np
import pygame as py

from contextlib import contextmanager


class Environment:

    RGB_BLACK_CODE = (0 , 0 , 0)
    RGB_WHITE_CODE = (255, 255, 255)
    RGB_ORANGE_CODE = (255, 140, 0)
    RGB_GREEN_CODE = (0, 255, 0)
    RGB_BLUE_CODE = (0, 0, 255)
    RGB_RED_CODE = (255, 0, 0)

    def __init__(self, c_space_dimensions: np.array, c_space_obstacles: np.array, car_size: np.array):
        """Base environment for the application."""
        self.c_space_dimensions = c_space_dimensions
        self.c_space_obstacles = c_space_obstacles

        self.screen = py.display.set_mode(c_space_dimensions)
        self.screen.fill(self.RGB_WHITE_CODE)

        self.car = py.Surface(car_size)
        self.car.set_colorkey(self.RGB_WHITE_CODE)
        self.car.fill(self.RGB_RED_CODE)

        self.draw_obstacles()

    def draw_obstacles(self):
        """Draw the obstacles in the application."""
        for obstacle in self.c_space_obstacles:
            obstacle_position, obstacle_size = obstacle.position, obstacle.size
            py.draw.rect(self.screen, self.RGB_BLUE_CODE, py.Rect(*obstacle_position, *obstacle_size))

        py.display.update()

    def draw_path(self, nodes, edges, shortest_path, draw_rrt=True):
        """Draw the rrt path in the application."""
        if draw_rrt:
            for node in nodes:
                py.draw.circle(self.screen, self.RGB_ORANGE_CODE, node, 2)

            for edge in edges:
                start_pos, end_pos = nodes[edge[0]], nodes[edge[1]]
                py.draw.line(self.screen, self.RGB_ORANGE_CODE, start_pos, end_pos, width=2)

        for shortest_path_edge in shortest_path:
            start_pos, end_pos = nodes[shortest_path_edge[0]], nodes[shortest_path_edge[1]]
            py.draw.line(self.screen, self.RGB_GREEN_CODE, start_pos, end_pos, width=2)

    def draw_coordinates(self, coordinates):
        """ """
        for coordinate in coordinates:
            py.draw.circle(self.screen, self.RGB_GREEN_CODE, coordinate, 4)

    def draw_vehicle(self, x_pos=0.0, y_pos=0.0, phi_steering_angle=0.0):
        """ """
        new_car = py.transform.rotate(self.car, phi_steering_angle)

        car_rectangle = new_car.get_rect()
        car_rectangle.center = (x_pos, y_pos)

        self.screen.blit(new_car, car_rectangle)
        py.display.flip()

    @contextmanager
    def in_progress(self):
        """Show that the application is calculating by changing the background color."""
        self.update_background(self.RGB_ORANGE_CODE)
        yield
        self.update_background(self.RGB_WHITE_CODE)

    def update_background(self, color):
        """Update the background color."""
        self.screen.fill(color)
        self.draw_obstacles()
        py.display.update()
