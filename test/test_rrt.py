import matplotlib.pyplot as plt
import matplotlib.patches as patches

import numpy as np

from map_environment.obstacles import Rectangle
from global_planner.rrt import RRT

if __name__ == '__main__':
    shelf_1 = Rectangle(np.array([50, 50]), np.array([20, 200]))
    shelf_2 = Rectangle(np.array([100, 50]), np.array([20, 200]))
    shelf_3 = Rectangle(np.array([150, 50]), np.array([20, 200]))
    shelf_4 = Rectangle(np.array([200, 50]), np.array([20, 200]))
    shelf_5 = Rectangle(np.array([250, 50]), np.array([20, 200]))

    c_space_dimension_boundaries = np.array([500, 500], dtype='int')
    c_space_obstacle_points = np.array([shelf_1, shelf_2, shelf_3, shelf_4, shelf_5])

    starting_point = np.array([0, 0], dtype='int')
    ending_point = np.array([175, 375], dtype='int')

    rrt_global_planner = RRT(c_space_dimension_boundaries, c_space_obstacle_points)
    rrt_nodes, rrt_edges = rrt_global_planner.build_roadmap(starting_point, ending_point, intermediate_goal_check=True)
    shortest_path_edges = rrt_global_planner.generate_shortest_path(starting_point, ending_point)

    # plot the beginning and ending point
    fig, ax = plt.subplots()
    ax.scatter(starting_point[0], starting_point[1], s=10, c="red")
    ax.scatter(ending_point[0], ending_point[1], s=10, c="red")

    # plot the obstacles
    for obstacle in rrt_global_planner.c_space_obstacles:
        position, size = obstacle.position, obstacle.size
        ax.add_patch(patches.Rectangle(position, *size))

    # plot the nodes as points in the graph
    plt.scatter(rrt_nodes[:, 0], rrt_nodes[:, 1], s=2, c="green")

    # plot the edges as lines in the graph
    for edge in rrt_edges:
        start_node, stop_node = rrt_nodes[edge[0]], rrt_nodes[edge[1]]

        x = [start_node[0], stop_node[0]]
        y = [start_node[1], stop_node[1]]
        plt.plot(x, y, color='green', linewidth=1)

    # plot the edges of the shortest path as lines in the graph
    for edge in shortest_path_edges:
        start_node, stop_node = rrt_nodes[edge[0]], rrt_nodes[edge[1]]

        x = [start_node[0], stop_node[0]]
        y = [start_node[1], stop_node[1]]
        plt.plot(x, y, color='red', linestyle='--', linewidth=1)

    # set limits according to dimension boundaries
    plt.xlim(0, c_space_dimension_boundaries[0])
    plt.ylim(0, c_space_dimension_boundaries[1])

    plt.grid(color='green', linestyle='--', linewidth=0.5)
    plt.show()
