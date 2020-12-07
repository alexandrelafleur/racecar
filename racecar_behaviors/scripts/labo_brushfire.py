#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from nav_msgs.srv import GetMap
from libbehaviors import occupancy_grid_to_maze, brushfire
from astar import pfind_new

def meter_to_cells((x_m, y_m)):
    resolution = 0.1
    corner_x = -8.98
    corner_y = -3.68

    x_cell = int((y_m- corner_y) / resolution) 
    y_cell = int((x_m - corner_x) / resolution)
    return x_cell, y_cell

def get_grid():
    rospy.init_node('brushfire')
    prefix = "racecar"
    rospy.wait_for_service(prefix + '/get_map')
    try:
        get_map = rospy.ServiceProxy(prefix + '/get_map', GetMap)
        response = get_map()
    except (rospy.ServiceException) as e:
        print ("Service call failed: %s%e")
        return

    rospy.loginfo("Got map=%dx%d resolution=%f", response.map.info.height,
                  response.map.info.width, response.map.info.resolution)
    
    grid = np.reshape(response.map.data, [
                      response.map.info.height, response.map.info.width])
    return grid

def get_path(x, y, id):
    grid = get_grid()
    maze = occupancy_grid_to_maze(grid)
    maze_list = maze.tolist()
    brushfire_map = brushfire(maze)

    start_cells = meter_to_cells((0,0))
    end_cells = meter_to_cells((x,y))
    path = pfind_new(maze_list, start_cells, end_cells, brushfire_map)

    print("start cells", start_cells)
    print("end cells", end_cells)

    print("start value is:", maze_list[start_cells[0]][start_cells[1]])
    print("end value is:", maze_list[end_cells[0]][end_cells[1]])

    print("path")
    print(path)

    # Export brusfire map for visualization
    # Adjust color: 0 (black) = obstacle, 10-255 (white) = safest cells
    maximum = np.amax(brushfire_map)
    if maximum > 1:
        mask = brushfire_map == 1
        brushfire_map = brushfire_map.astype(
            float) / float(maximum) * 225.0 + 30.0
        brushfire_map[mask] = 0
        # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
        cv2.imwrite('brushfire.bmp', cv2.transpose(cv2.flip(brushfire_map, -1)))
        rospy.loginfo("Exported brushfire.bmp")
    else:
        rospy.loginfo("brushfire failed! Is brusfire implemented?")

    # Example to show grid with same color than RVIZ
    grid[grid == -1] = 89
    grid[grid == 0] = 178
    grid[grid == 100] = 0
    # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
    cv2.imwrite('map.bmp', cv2.transpose(cv2.flip(grid, -1)))
    rospy.loginfo("Exported map.bmp")

    # Example to show grid with same color than RVIZ
    maze = np.array(maze_list)
    maze[maze == -1] = 80
    maze[maze == 0] = 120
    maze[start_cells[0]][start_cells[1]] = 0
    maze[end_cells[0]][end_cells[1]] = 0
    if path:
        for i, step in enumerate(path):
            if i != 0 and i!=len(path)-1:
                maze[step[0]][step[1]] = 255

    cv2.imwrite('maze.bmp', cv2.transpose(cv2.flip(maze, -1)))
    rospy.loginfo("Exported maze.bmp")



def main():
    x = 8.4
    y = 2.1
    id = 0
    get_path(x, y, id)


if __name__ == '__main__':
    main()
