
import numpy as np

class Rectangle:

    MARGIN = 5

    def __init__(self, base_corner: np.array, size: np.array):
        """Initialise an obstacle for the environment.
        :param base_corner: lower left corner point of the obstacle.
        :param size: xy size of the obstacle.
        """
        self.position = base_corner
        self.size = size

    def boundary_points(self):
        """Return xy boundaries of the rectangle obstacle."""
        x_min, x_max = self.position[0], self.position[0] + self.size[0]
        y_min, y_max = self.position[1], self.position[1] + self.size[1]

        return x_min, y_min, x_max, y_max

    def is_point_in_shape_area(self, point):
        """Check if a given point is within the boundaries of the obstacle.
        :param point:
            xy point to check.

        :return:
            True if the point is within the obstacle boundaries, else False.
        """
        x_min, x_max = self.position[0] - self.MARGIN, self.position[0] + self.size[0] + self.MARGIN
        y_min, y_max = self.position[1] - self.MARGIN, self.position[1] + self.size[1] + self.MARGIN

        if x_min <= point[0] <= x_max and y_min <= point[1] <= y_max:
            return True

        return False

class Circle:

    MARGIN = 5

    def __init__(self, center: np.array, size: float):
        """Initialise an obstacle for the environment.
        :param center: center of the pillar.
        :param size: radius of the pillar
        """
        self.center = center
        self.size = size

    def boundary_points(self):
        """Return center x, y  and size of the circle."""
        return self.center[0], self.center[1], self.size

    def is_point_in_shape_area(self, point):
        """Check if a given point is within the boundaries of the obstacle.
        :param point:
            xy point to check.
        :return:
            True if the point is within the obstacle boundaries, else False.
        """
        if np.linalg.norm(point - self.center) + self.MARGIN < self.size:
            return True

        return False