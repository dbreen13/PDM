import matplotlib.pyplot as plt
import numpy as np
from rrt import RRT

if __name__ == '__main__':
    c_space_dimension_boundaries = np.array([200, 400], dtype='int')

    obstacle_1 = np.array([[50, 50], [50, 100], [75, 125], [100, 100], [100, 50], [75, 25]])
    obstacle_2 = np.array([[10, 150], [10, 175], [75, 175], [180, 175], [180, 150], [75, 150]])

    c_space_obstacle_points = np.array([obstacle_1, obstacle_2])

    starting_point = np.array([0, 0], dtype='int')
    ending_point = np.array([175, 375], dtype='int')

    rrt_global_planner = RRT(c_space_dimension_boundaries, c_space_obstacle_points)
    rrt_nodes, rrt_edges = rrt_global_planner.build_roadmap(starting_point, ending_point, intermediate_goal_check=True)
    shortest_path_edges = rrt_global_planner.generate_shortest_path(starting_point, ending_point)

    # plot the beginning and ending point
    plt.scatter(starting_point[0], starting_point[1], s=10, c="red")
    plt.scatter(ending_point[0], ending_point[1], s=10, c="red")

    # plot the obstacles
    for polygon_obstacle in rrt_global_planner.polygon_c_space_obstacles:
        plt.plot(*polygon_obstacle.exterior.xy, color='blue', linewidth=2)

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
