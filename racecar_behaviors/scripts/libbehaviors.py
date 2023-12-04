#!/usr/bin/env python

import rospy
import cv2
import tf
import numpy as np
from tf.transformations import euler_from_quaternion


def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message:
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw


def multiply_transforms(trans1, rot1, trans2, rot2):
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


def brushfire(occupancyGrid):
    mapOfWorld = np.zeros(occupancyGrid.shape, dtype=int)

    mapOfWorld[occupancyGrid == 100] = 1  # obstacles
    mapOfWorld[occupancyGrid == -1] = 1  # unknowns

    nRows = 210
    nCols = 397
    use8CellWindow = False
    # do brushfire algorithm here

    a = 0
    while 0 in mapOfWorld:
        a = a + 1
        for iRow in range(nRows):
            for iCol in range(nCols):
                if mapOfWorld[iRow][iCol] == a:
                    if mapOfWorld[iRow - 1][iCol] == 0:
                        mapOfWorld[iRow - 1][iCol] = a + 1
                    if mapOfWorld[iRow][iCol - 1] == 0:
                        mapOfWorld[iRow][iCol - 1] = a + 1
                    if mapOfWorld[iRow + 1][iCol] == 0:
                        mapOfWorld[iRow + 1][iCol] = a + 1
                    if mapOfWorld[iRow][iCol + 1] == 0:
                        mapOfWorld[iRow][iCol + 1] = a + 1
                    if use8CellWindow:
                        if mapOfWorld[iRow - 1][iCol - 1] == 0:
                            mapOfWorld[iRow - 1][iCol - 1] = a + 1
                        if mapOfWorld[iRow - 1][iCol + 1] == 0:
                            mapOfWorld[iRow - 1][iCol + 1] = a + 1
                        if mapOfWorld[iRow + 1][iCol - 1] == 0:
                            mapOfWorld[iRow + 1][iCol - 1] = a + 1
                        if mapOfWorld[iRow + 1][iCol + 1] == 0:
                            mapOfWorld[iRow + 1][iCol + 1] = a + 1

    # brushfire: -1 = obstacle or unknown, safer cells have higher value)
    return mapOfWorld
