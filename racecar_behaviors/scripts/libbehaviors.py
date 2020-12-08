#!/usr/bin/env python3

import rospy
import cv2
import tf
import numpy as np
from tf.transformations import euler_from_quaternion


def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message:
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion(
        [quat.x, quat.y, quat.z, quat.w])
    return yaw

def multiply_transforms(arg1, arg2):
    trans1 = arg1[0]
    rot1 = arg1[1]
    trans2 = arg2[0]
    rot2 = arg2[1]
    
    trans1_mat = tf.transformations.translation_matrix(trans1)
    rot1_mat = tf.transformations.quaternion_matrix(rot1)
    mat1 = np.dot(trans1_mat, rot1_mat)

    trans2_mat = tf.transformations.translation_matrix(trans2)
    rot2_mat = tf.transformations.quaternion_matrix(rot2)
    mat2 = np.dot(trans2_mat, rot2_mat)

    mat3 = np.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)

    return (trans3, rot3)

def occupancy_grid_to_maze(occupancyGrid):
    maze = np.zeros(occupancyGrid.shape, dtype=int)
    maze[occupancyGrid == 100] = -1  # obstacles
    maze[occupancyGrid == -1] = -1

    return maze


def brushfire(maze):
    mapOfWorld = maze

    # do brushfire algorithm here
    i = -1
    it = 2
    first = True
    # Loop qui change les 0 pour un chiffre si c'est dans le range
    while 1:
        rows, cols = np.where(mapOfWorld == i)
        for j in range(rows.size):
            if rows[j] + 1 < mapOfWorld.shape[0]:
                if mapOfWorld[rows[j] + 1, cols[j]] == 0:
                    mapOfWorld[rows[j] + 1, cols[j]] = i + it
            if rows[j] - 1 >= 0:
                if mapOfWorld[rows[j] - 1, cols[j]] == 0:
                    mapOfWorld[rows[j] - 1, cols[j]] = i + it
            if cols[j] + 1 < mapOfWorld.shape[1]:
                if mapOfWorld[rows[j], cols[j] + 1] == 0:
                    mapOfWorld[rows[j], cols[j] + 1] = i + it
            if cols[j] - 1 >= 0:
                if mapOfWorld[rows[j], cols[j] - 1] == 0:
                    mapOfWorld[rows[j], cols[j] - 1] = i + it
        # Ajustement des variables
        if first:
            i = i + 2
            it = 1
            first = False
        else:
            i = i + 1
        # Si il y a encore des zeros, continue, sinon arrete
        zrows, zcols = np.where(mapOfWorld == 0)
        if zrows.size == 0 and zcols.size == 0:
            break

    # brushfire: -1 = obstacle or unknown, safer cells have higher value)
    return mapOfWorld
