#!/usr/bin/env python


import numpy as np
from libbehaviors import brushfire

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end, brushfire_map):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""
    # Allow diagonal movement
    print("Astar starting...")
    allow_diagonal_movement = 0

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    i = 1
    first_five = 1
    while len(open_list) > 0:
        # print(len(open_list))
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
                print("hello")
                print(path)
            return path[::-1]  # Return reversed path

        # Generate children
        children = []
        adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0),)
        if allow_diagonal_movement:
            adjacent_squares = ((0, -1), (0, 1), (-1, 0),
                                (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1),)
        for new_position in adjacent_squares:

            # Get node position
            node_position = (
                current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) - 1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            broke = 0
            for closed_child in closed_list:
                if child == closed_child:
                    broke = 1
                    break
            if broke and not first_five:
                continue
            # Create the f, g, and h values
            child.g = current_node.g + \
                get_cost(maze, current_node.position, end_node.position, brushfire_map)
            child.h = (((child.position[0] - end_node.position[0]) ** 2) + (
                (child.position[1] - end_node.position[1]) ** 2)) ** (1/2)
            child.f = child.g + child.h

            # Child is already in the open list
            broke = 0
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    broke = 1
                    break
            if broke and not first_five:
                continue

            # Add the child to the open list
            open_list.append(child)
        i = i+1
        if i == 6:
            first_five = 0


def get_cost(maze, currpos, nextpos, bfmap):
    diff = bfmap[currpos[0], currpos[1]]-bfmap[nextpos[0], nextpos[1]]
    if diff < (-1):
        cost = 1
    elif diff == (-1):
        cost = 10
    elif diff == 0:
        cost = 50
    elif diff == 1:
        cost = 50
    else:
        cost = 50
    return cost


def main():

    maze = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, -1, -1, -1, 0, 0, 0, 0, 0],
            [-1, -1, -1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, -1, -1, -1, 0],
            [0, 0, 0, 0, 0, 0, 0, -1, -1, -1],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, -1, -1, -1, 0, 0, 0, 0, 0, 0],
            [-1, -1, -1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    start = (9, 4)
    end = (0, 0)

    mazeNp = np.array(maze)
    bfmap = brushfire(mazeNp)
    path = astar(maze, start, end, bfmap)
    print(bfmap)
    for i in range(0, len(path)):
        curr_space = path[i]
        mazeNp[curr_space[0], curr_space[1]] = i+1
    print(mazeNp)


if __name__ == '__main__':
    main()
