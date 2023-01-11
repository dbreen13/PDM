import numpy as np
import pygame

class FailedBuildingRoadmap(Exception):
    """Base class to generate custom exception if building of roadmap failed."""

    def __init__(self, error_message):
        """ Construct custom error with custom error message
        :param error_message: The custom error message
        """
        super().__init__(error_message)


class FailedToGeneratePath(Exception):
    """Base class to generate custom exception if generating path from edges."""

    def __init__(self, error_message):
        """ Construct custom error with custom error message
        :param error_message: The custom error message
        """
        super().__init__(error_message)


class RRT:
    """Base class to generate a global path using the rrt algorithm"""

    def __init__(self, c_space_dimensions: np.array, c_space_obstacles: np.array):
        """ Create a rrt object which can be used to generate paths in a given configuration space
        :param c_space_dimensions: The outer boundaries of the configuration space
        :param c_space_obstacles: Array of the obstacle contours in the configuration space
        """
        self.c_space_dimensions = c_space_dimensions
        self.c_space_obstacles = c_space_obstacles

        self.nodes = np.empty((0, 2), dtype='int')
        self.edges = np.empty((0, 2), dtype='int')

    def build_roadmap(self, q_init, q_goal, max_iterations=5000, intermediate_goal_check=False, update_screen=True):
        """ Build the rrt roadmap tree from a given start to given end configuration
        :param q_init: Starting position of the tree
        :param q_goal: Ending position for the tree
        :param max_iterations: Max number of expansions
        :param intermediate_goal_check: Check between random samples if goal is reachable
        :param update_screen: Update the pygame screen between iterations

        :return:
            Two arrays of the created nodes and edges
        """
        if self.is_node_in_obstacle_space(q_init):
            FailedToGeneratePath("Given initial configuration is in obstacle space.")

        if self.is_node_in_obstacle_space(q_goal):
            FailedToGeneratePath("Given goal configuration is in obstacle space.")

        self.nodes = np.empty((0, 2), dtype='int')
        self.edges = np.empty((0, 2), dtype='int')

        # add starting point to nodes
        self.nodes = np.vstack((self.nodes, q_init))

        for iteration in range(max_iterations):
            new_node = self.generate_random_node()

            if new_node.tolist() in self.nodes.tolist():
                continue

            if self.is_node_in_obstacle_space(new_node):
                continue

            closest_node = self.closest_node(new_node, self.nodes)
            new_edge = np.array([closest_node, new_node])

            if self.is_edge_in_obstacle_space(new_edge):
                continue

            self.nodes = np.vstack((self.nodes, new_node))

            closest_node_idx = np.where(np.all(self.nodes == closest_node, axis=1))[0][0]
            created_node_idx = np.where(np.all(self.nodes == new_node, axis=1))[0][0]

            new_edge_indices = np.array([closest_node_idx, created_node_idx])
            self.edges = np.vstack((self.edges, new_edge_indices))

            if q_goal.tolist() in self.nodes.tolist():
                break

            if intermediate_goal_check and self.is_goal_reachable(q_goal):
                closest_node = self.closest_node(q_goal, self.nodes)
                closest_node_idx = np.where(np.all(self.nodes == closest_node, axis=1))[0][0]

                self.nodes = np.vstack((self.nodes, q_goal))
                created_node_idx = np.where(np.all(self.nodes == q_goal, axis=1))[0][0]

                new_edge_indices = np.array([closest_node_idx, created_node_idx])
                self.edges = np.vstack((self.edges, new_edge_indices))

                break

            if update_screen:
                pygame.display.update()

        if q_goal.tolist() not in self.nodes.tolist():
            if not self.is_goal_reachable(q_goal):
                raise FailedBuildingRoadmap('Could connect roadmap to the given goal position.')

            closest_node = self.closest_node(q_goal, self.nodes)
            closest_node_idx = np.where(np.all(self.nodes == closest_node, axis=1))[0][0]

            self.nodes = np.vstack((self.nodes, q_goal))
            created_node_idx = np.where(np.all(self.nodes == q_goal, axis=1))[0][0]

            new_edge_indices = np.array([closest_node_idx, created_node_idx])
            self.edges = np.vstack((self.edges, new_edge_indices))

        return self.nodes, self.edges

    def generate_shortest_path(self, q_init, q_goal):
        """Find the shortest rrt path in the created roadmap.
        :param q_init: Starting position of the tree
        :param q_goal: Ending position for the tree

        :return:
            A list of edges from start to goal
        """
        if q_init.tolist() not in self.nodes.tolist():
            raise FailedToGeneratePath("Start configuration not in list of nodes,")

        if q_goal.tolist() not in self.nodes.tolist():
            raise FailedToGeneratePath("Goal configuration not in list of nodes,")

        path = np.empty((0, 2), dtype='int')
        for edge in reversed(self.edges):
            stop_node_edge = self.nodes[edge[1]]

            if np.array_equal(stop_node_edge, q_goal):
                path = np.vstack((edge, path))

            if path.shape[0] != 0:
                open_node_path = self.nodes[path[0][0]]
                if np.array_equal(stop_node_edge, open_node_path):
                    path = np.vstack((edge, path))

        return path

    def generate_random_node(self):
        """ Create a new node in the configuration space.

        :return:
            The coordinate of the new node
        """
        return np.random.randint(self.c_space_dimensions, size=2, dtype='int')

    def is_node_in_obstacle_space(self, node):
        """ Check if a node is in obstacle space or free space.
        :param node:
            Coordinate of the new node.

        :return:
            True if node is in obstacle space, else False.
        """
        return any(obstacle.is_point_in_shape_area(node) for obstacle in self.c_space_obstacles)

    def is_edge_in_obstacle_space(self, edge):
        """ Discretize linearly between two nodes and check if intermediate nodes are in obstacle space or free space.
        :param edge:
            The edge te check.

        :return:
            True if edge is in obstacle space, else False.
        """
        intermediate_nodes = self.discretize_edge(edge)

        for intermediate_node in intermediate_nodes:
            if any(obstacle.is_point_in_shape_area(intermediate_node) for obstacle in self.c_space_obstacles):
                return True

        return False

    def is_goal_reachable(self, q_goal):
        """ Check if the goal can be reached with the current set of nodes.
        :param q_goal:
            The goal configuration to check for reachability.

        :return:
            True if goal is reachable, else False.
        """
        closest_node = self.closest_node(q_goal, self.nodes)
        new_edge = np.array([closest_node, q_goal])

        if self.is_edge_in_obstacle_space(new_edge):
            return False

        return True

    @staticmethod
    def closest_node(current_node, all_nodes):
        """ Find the closest node in the list of nodes to a given node.
        :param current_node:
            The node to check against the list of nodes.
        :param all_nodes:

        :return:
        """
        idx_to_closest_node = np.argmin([np.linalg.norm(current_node - node) for node in all_nodes])
        return all_nodes[idx_to_closest_node]

    @staticmethod
    def discretize_edge(edge, points=50):
        """ Linearly discretize an edge into multiple points.
        :param points:
            Amount of points used in the discretize process.
        :param edge:
            The edge used in the discretize process.

        :return:
            Set of intermediate nodes.
        """
        start_point, end_point = edge[0], edge[1]
        intermediate_nodes = start_point + np.linspace(0, 1, points)[..., np.newaxis] * (end_point - start_point)

        return intermediate_nodes

    @staticmethod
    def coordinates_from_shortest_path(nodes, shortest_path):
        """ """
        coordinates = []
        for shortest_path_edge in shortest_path:
            coordinate_edge = nodes[shortest_path_edge[0]], nodes[shortest_path_edge[1]]
            amount_of_points = int(np.linalg.norm(coordinate_edge[1] - coordinate_edge[0]) / 10)

            discretize_edge = RRT.discretize_edge(coordinate_edge, points=amount_of_points)
            discretize_edge = np.round(discretize_edge)

            coordinates.extend(discretize_edge)

        return np.array(coordinates)