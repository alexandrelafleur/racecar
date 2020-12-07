#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from nav_msgs.srv import GetMap
from libbehaviors import occupancy_grid_to_maze, brushfire
from astar import astar


def main():
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
    start = (100, 50)
    end = (101, 50)
    
    grid = np.reshape(response.map.data, [
                      response.map.info.height, response.map.info.width])
    maze = occupancy_grid_to_maze(grid)
    maze_list = maze.tolist()
    brushfireMap = brushfire(maze)
    # path = astar(maze.tolist(), start, end)

    itty_bitty_maze = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, -1, -1, -1, 0, 0, 0, 0, 0],
            [-1, -1, -1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, -1, -1, -1, 0],
            [0, 0, 0, 0, 0, 0, 0, -1, -1, -1],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, -1, -1, -1, 0, 0, 0, 0, 0, 0],
            [-1, -1, -1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
    small_maze = maze_list[100:102]
    #print(maze_list)
    path = astar(small_maze, (0,61), (0,88))

    # for i, row in enumerate(small_maze):
    #     for j,cell in enumerate(row):
    #         if cell == 0:
    #             print(i,j)

    # print("grid")
    # print (grid)

    # print("maze")
    # print (maze)

    # # print("maze.tolist()")
    # # print (maze.tolist())

    # print("brushfireMap")
    # print (brushfireMap)

    print("path")
    print(path)

 

 

    
    mazeNp = np.array(maze)

    # for i in range(0, len(path)):
    #     curr_space = path[i]
    #     mazeNp[curr_space[0], curr_space[1]] = i+1
    # print(mazeNp)



    # Export brusfire map for visualization
    # Adjust color: 0 (black) = obstacle, 10-255 (white) = safest cells
    maximum = np.amax(brushfireMap)
    if maximum > 1:
        mask = brushfireMap == 1
        brushfireMap = brushfireMap.astype(
            float) / float(maximum) * 225.0 + 30.0
        brushfireMap[mask] = 0
        # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
        cv2.imwrite('brushfire.bmp', cv2.transpose(cv2.flip(brushfireMap, -1)))
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
    maze[maze == -1] = 120
    maze[maze == 0] = 80

    print("color",maze[110][50])
    #maze[maze == 100] = 0
    cv2.imwrite('maze.bmp', cv2.transpose(cv2.flip(maze, -1)))
    rospy.loginfo("Exported maze.bmp")

        # Example to show grid with same color than RVIZ
    itty_bitty_maze = np.array(itty_bitty_maze)
    itty_bitty_maze[itty_bitty_maze == -1] = 120
    itty_bitty_maze[itty_bitty_maze == 0] = 80
    cv2.imwrite('itty_bitty_maze.bmp', cv2.transpose(cv2.flip(itty_bitty_maze, -1)))
    rospy.loginfo("Exported itty_bitty_maze.bmp")

    small_maze = np.array(small_maze)
    small_maze[small_maze == -1] = 120
    small_maze[small_maze == 0] = 80
    cv2.imwrite('small_maze.bmp', cv2.transpose(cv2.flip(small_maze, -1)))
    rospy.loginfo("Exported small_maze.bmp")

if __name__ == '__main__':
    main()
