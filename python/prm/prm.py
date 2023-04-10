"""
Implementation of the PRM algorithm.

@author: kjartan@tec.mx with help from GitHub copilot
"""
from typing import Tuple, Any

import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import doctest
import random
import math
import time
import sys


class PRM:
    """
    Implementation of the Probabilistic Road Map algorithm.

    Attributes
    ----------
        start (tuple): The start configuration of the robot.
        goal (tuple): The goal configuration of the robot.
        obstacles (list): A list of obstacles in the environment.
        num_nodes (int): The number of nodes to generate in the roadmap.
        roadmap (nx.Graph): The roadmap of the environment.
        path (list): The path from start to goal.


    Methods
    -------
        plan(start, goal)
            Plan a path from start to goal.
        _create_graph()
            Create the roadmap of the environment.
        _path_is_clear(node1, node2)
            Check if the path between two nodes is clear.
        _distance(node1, node2)
            Compute the distance between two nodes.

    Tests
    ------
    >>> prm = PRM(np.zeros((10, 10)), 10)
    >>> prm.plan((0, 0), (9, 9))
    True
    """

    def __init__(self, map: np.ndarray, num_nodes: int):
        """
        Initialize the PRM algorithm.

        Args:
            map (np.ndarray): The occupancy grid map of the environment.
            num_nodes (int): The number of nodes to generate in the roadmap.

        """
        self.start = None
        self.goal = None
        self.obstacles = []
        self.num_nodes = num_nodes
        self.roadmap = nx.Graph()
        self.path = []
        self.map = map

        self._create_graph()

    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> bool:
        """
        Plan a path from start to goal.

        Arguments
        ----------
            start (tuple): The start position of the robot.
            goal (tuple): The goal position of the robot.

        Returns
        -------
            bool: True if a path is found, False otherwise.

        """
        self.start = start
        self.goal = goal

        # Add start and goal to the roadmap
        self.roadmap.add_node('start', pos=start)
        self.roadmap.add_node('goal', pos=goal)
        for i in range(self.num_nodes):
            if self._path_is_clear('start', i)[0]:
                self.roadmap.add_edge('start', i, weight=self._distance('start', i))
            if self._path_is_clear('goal', i)[0]:
                self.roadmap.add_edge('goal', i, weight=self._distance('goal', i))

        # Find the shortest path from start to goal
        try:
            self.path = nx.shortest_path(self.roadmap, 'start', 'goal', weight='weight')
            return True
        except nx.NetworkXNoPath:
            return False

    def _create_graph(self):
        """
        Create the roadmap of the environment.
        """

        height, width = self.map.shape

        # Add nodes to the roadmap
        for i in range(self.num_nodes):
            in_free_space = False
            while not in_free_space:
                pos = (random.randint(0, width - 1), random.randint(0, height - 1))
                if self.map[pos] == 0:
                    in_free_space = True
            self.roadmap.add_node(i, pos=pos)

        # Add edges to the roadmap
        for i in range(self.num_nodes):
            for j in range(i + 1, self.num_nodes):
                ok, dist = self._path_is_clear(i, j)
                if ok: self.roadmap.add_edge(i, j, weight=dist)

    def _path_is_clear(self, node1: Any, node2: Any) -> Tuple[bool, float]:
        """
        Check if the path between two nodes is clear.

        Arguments
        ----------
            node1 (Any): The first node.
            node2 (Any): The second node.

        Returns
        -------
            bool: True if the path is clear, False otherwise.
            dist: The distance between the two nodes.

        """
        pos1 = self.roadmap.nodes[node1]['pos']
        pos2 = self.roadmap.nodes[node2]['pos']
        dist = self._distance(node1, node2)
        num_steps = int(dist)
        step_size = 1.0 / num_steps
        for i in range(num_steps):
            beta = i * step_size
            pos = (round(pos1[0] * (1 - beta) + pos2[0] * beta), round(pos1[1] * (1 - beta) + pos2[1] * beta))
            if self.map[pos] == 1:
                return (False, np.inf)
        return (True, dist)

    def _distance(self, node1: Any, node2: Any) -> float:
        """
        Compute the distance between two nodes.

        Arguments
        ---------
        node1 (Any): The first node.
        node2 (Any): The second node.

        Returns
        -------
        float: The distance between the two nodes.
        """
        pos1 = self.roadmap.nodes[node1]['pos']
        pos2 = self.roadmap.nodes[node2]['pos']
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)


def main(num_nodes: int, mapfile: str, start: Tuple[int, int], goal: Tuple[int, int]) -> None:
    """
    Run the PRM algorithm.
    """
    # Load the map
    map = np.load(mapfile)

    # Create the PRM object
    prm = PRM(map, num_nodes)

    # Plan a path from start to goal
    success = prm.plan(start, goal)
    print('Path found:', success)

    # Plot the path
    if success:
        fig, ax = plt.subplots()
        ax.imshow(map, cmap='Greys', origin='lower')
        nx.draw(prm.roadmap, pos=nx.get_node_attributes(prm.roadmap, 'pos'), ax=ax, node_size=5, width=0.5)
        ax.plot([start[0], goal[0]], [start[1], goal[1]], 'r-')
        plt.show()


if __name__ == '__main__':

    # Default values
    num_nodes = 100
    map = 'map.npy'
    if len(sys.argv) > 1:
        if sys.argv[1] == '--test':
            import doctest

            doctest.testmod()
            sys.exit()

        num_nodes = int(sys.argv[1])
    if len(sys.argv) > 2:
        map = np.load(sys.argv[2])

    main(num_nodes, map, (0, 0), (9, 9))
