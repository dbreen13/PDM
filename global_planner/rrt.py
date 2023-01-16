import numpy as np
import pygame as py

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

    MAX_ITERATIONS = 10000

    def __init__(self, c_space_dimensions: np.array, c_space_obstacles: np.array, c_space_pillars: np.array):
        """ Create a rrt object which can be used to generate paths in a given configuration space
        :param c_space_dimensions: The outer boundaries of the configuration space
        :param c_space_obstacles: Array of the obstacle contours in the configuration space
        """
        self.c_space_dimensions = c_space_dimensions
        self.c_space_obstacles = c_space_obstacles
        self.c_space_pillars = c_space_pillars

        self.nodes = np.empty((0, 2), dtype='int')
        self.edges = np.empty((0, 2), dtype='int')

    def build_roadmap(self, q_init, q_goal, rewire=False, intermediate_goal_check=False, update_screen=True):
        """ Build the rrt roadmap tree from a given start to given end configuration
        :param q_init: Starting position of the tree
        :param q_goal: Ending position for the tree
        :param rewire: extend algorithm to rrt star by rewiring at the end
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

        for iteration in range(self.MAX_ITERATIONS):
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
                py.display.update()

        if q_goal.tolist() not in self.nodes.tolist():
            if not self.is_goal_reachable(q_goal):
                raise FailedBuildingRoadmap('Could connect roadmap to the given goal position.')

            closest_node = self.closest_node(q_goal, self.nodes)
            closest_node_idx = np.where(np.all(self.nodes == closest_node, axis=1))[0][0]

            self.nodes = np.vstack((self.nodes, q_goal))
            created_node_idx = np.where(np.all(self.nodes == q_goal, axis=1))[0][0]

            new_edge_indices = np.array([closest_node_idx, created_node_idx])
            self.edges = np.vstack((self.edges, new_edge_indices))

        if rewire:
            print(f'LOGGER: Done building roadmap, nodes: {self.nodes.shape[0]} , start rewiring path...')
            self.rewire_nodes(q_goal, update_screen)

        return self.nodes, self.edges

    def rewire_nodes(self, q_goal, update_screen=True):
        """Rewire the set of nodes with new edges to define the minimal spanning tree using Prim's algorithm.
        :param q_goal: Ending position for the tree
        :param update_screen: Update the pygame screen between iterations
        """
        adjacency_matrix = self.generate_adjacency_matrix_of_nodes()
        selected_node = np.array([False] * adjacency_matrix.shape[0])

        new_edges = np.empty((0, 2), dtype='int')
        amount_of_nodes = adjacency_matrix.shape[0]

        index_of_endpoint = None
        if q_goal.tolist() not in self.nodes.tolist():
            index_of_endpoint = np.where(self.nodes == q_goal)[0][0]

        selected_node[0] = True
        while new_edges.shape[0] < amount_of_nodes - 1:
            optimal_row_node = 0
            optimal_col_node = 0

            minimum = 9e2
            for row_node in range(amount_of_nodes):
                if selected_node[row_node]:
                    for col_node in range(amount_of_nodes):
                        if not selected_node[col_node] and adjacency_matrix[row_node][col_node]:

                            if minimum > adjacency_matrix[row_node][col_node]:
                                minimum = adjacency_matrix[row_node][col_node]
                                optimal_row_node = row_node
                                optimal_col_node = col_node

                    if update_screen:
                        py.display.update()

            new_edge_indices = np.array([optimal_row_node, optimal_col_node])
            new_edges = np.vstack((new_edges, new_edge_indices))

            selected_node[optimal_col_node] = True

            if index_of_endpoint and selected_node[index_of_endpoint]:
                break

        self.edges = new_edges

    def generate_adjacency_matrix_of_nodes(self):
        """Define the adjacency matrix of the current nodes.

        :returns:
            The adjacency matrix of the current nodes
        """
        adjacency_matrix = []

        for selected_node in self.nodes:
            adjacency_list =  np.array([0.] * len(self.nodes))

            for node_idx, node in enumerate(self.nodes):
                if np.array_equal(selected_node, node):
                    continue

                new_edge = np.array([selected_node, node])
                if self.is_edge_in_obstacle_space(new_edge):
                    continue

                adjacency_list[node_idx] = int(np.linalg.norm(selected_node - node))
            adjacency_matrix.append(adjacency_list)

        return np.array(adjacency_matrix)

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

            if any(obstacle.is_point_in_shape_area(intermediate_node) for obstacle in self.c_space_pillars):
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
            All the available nodes

        :return:
            value of the node closest to the current node
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
    def coordinates_from_shortest_path(nodes, shortest_path, discretize_factor=0.1):
        """Define the shortest path coordinates based on the nodes, edges and discretize factor.
        :param nodes:
            The nodes of the roadmap
        :param shortest_path:
            The edges corresponding to the shortest path
        :param discretize_factor:
            Discretize factor to change amount of points between a line

        :returns:
            Returns the (x, y) coordinates of the shortest path
        """
        coordinates = []
        for shortest_path_edge in shortest_path:
            coordinate_edge = nodes[shortest_path_edge[0]], nodes[shortest_path_edge[1]]
            amount_of_points = int(np.linalg.norm(coordinate_edge[1] - coordinate_edge[0]) * discretize_factor)

            discretize_edge = RRT.discretize_edge(coordinate_edge, points=amount_of_points)
            discretize_edge = np.round(discretize_edge)

            coordinates.extend(discretize_edge)

        return np.array(coordinates)