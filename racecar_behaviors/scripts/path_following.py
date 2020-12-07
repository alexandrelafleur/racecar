#!/usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from labo_brushfire import get_path


class State:
    FOLLOW_PATH = 0
    CENTER_ON_OBJECT = 1
    WAIT_FOR_SCAN = 2
    DID_A_FULL_ONE_EIGHTY = 3
    REWIND = 4
    STOP = 5


class Blob:
    def __init__(self, x, y):
        self.abs_pos_x = x
        self.abs_pos_y = y


class PathFollowing:
    def __init__(self):
        self.twist = Twist()
        self.max_speed = rospy.get_param('~max_speed', 0.25)
        self.max_steering = rospy.get_param('~max_steering', 0.37)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber(
            'scan', LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(
            'odom', Odometry, self.odom_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber(
            'obstacle_detected', Bool, self.obstacle_callback, queue_size=1)
        self.blob_sub = rospy.Subscriber(
            'object_detected', Twist, self.blob_callback, queue_size=1)
        self.state_timer = rospy.Timer(
            rospy.Duration(0.05), self.state_machine_callback)

        # VARIABLES
        self.start_pos_x = 0
        self.start_pos_y = 0
        self.goal_pos_x = 13.5
        self.end_pos_x = 0
        self.goal_pos_y = 2.1
        self.delta_pos = 0.5
        self.blob_delta_pos = 3
        self.distance = 1.5
        self.dont_look_for_objects_no_more = False
        self.state = State.FOLLOW_PATH
        self.obstacle_detected = False
        self.R_obstacleDetected = False
        self.L_obstacleDetected = False
        self.reached_the_end = False
        self.visited_blobs = []

    def scan_callback(self, msg):
        # Because the lidar is oriented backward on the racecar,
        # if we want the middle value of the ranges to be forward:

        l2 = len(msg.ranges)/2
        ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]

        # Obstacle front?   l2+l2/8
        self.R_obstacleDetected = False
        self.L_obstacleDetected = False
        for i in range(3*l2/4 - l2/16, 3*l2/4 + l2/16):
            if np.isfinite(ranges[i]) and ranges[i] > 0 and ranges[i] < self.distance:
                self.R_obstacleDetected = True
                break
        for i in range(5*l2/4 - l2/16, 5*l2/4 + l2/16):
            if np.isfinite(ranges[i]) and ranges[i] > 0 and ranges[i] < self.distance:
                self.L_obstacleDetected = True
                break

        self.state_machine_callback(None)

    def odom_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.theta = msg.pose.pose.orientation.z
        # print("pos_x", self.pos_x)
        # and abs(self.pos_y - self.goal_pos_y) < self.delta_pos)
        if abs(self.pos_x - self.goal_pos_x) < self.delta_pos:
            if self.state == State.FOLLOW_PATH:
                if abs(self.theta) < 0.5:
                    print("DOING A 180")
                    self.state = State.DID_A_FULL_ONE_EIGHTY
                    self.reached_the_end = True

        # and abs(self.pos_y - self.goal_pos_y) < self.delta_pos)
        if abs(self.pos_x - self.start_pos_x) < self.delta_pos:
            if self.state == State.FOLLOW_PATH and self.reached_the_end:
                print("REACHED THE START")
                self.state = State.STOP

    def blob_callback(self, msg):
        print("found object")

        self.angle_blob = msg.angular.z
        self.distance_blob = msg.linear.x

        if self.state == State.CENTER_ON_OBJECT and self.distance_blob < 2.0:
            self.state = State.WAIT_FOR_SCAN
            print('state: Waiting for scan')

        elif self.distance_blob < 2.5:

            absolute_blob_pos_x = self.pos_x + self.distance_blob * \
                math.cos(self.angle_blob + self.theta)
            absolute_blob_pos_y = self.pos_y + self.distance_blob * \
                math.sin(self.angle_blob + self.theta)

            # check if seen before
            seenBefore = False
            for blob in self.visited_blobs:
                if abs(absolute_blob_pos_x - blob.abs_pos_x) < self.blob_delta_pos:
                    if abs(absolute_blob_pos_y - blob.abs_pos_y) < self.blob_delta_pos:
                        seenBefore = True
            if not seenBefore:
                self.visited_blobs.append(
                    Blob(absolute_blob_pos_x, absolute_blob_pos_y))

                if self.distance_blob < 2.5:
                    self.state = State.CENTER_ON_OBJECT
                    print('state: Trying to Center the object!')

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data
        if self.obstacle_detected:
            print("Obstacle :", msg.data)

    def state_machine_callback(self, state_timer):
        # print("state:", self.state)
        print(self.pos_x, self.pos_y)
        if self.state == State.CENTER_ON_OBJECT:
            self.mode_center_on_object()

        elif self.state == State.WAIT_FOR_SCAN:
            self.mode_wait_for_scan()

        elif self.state == State.FOLLOW_PATH:
            self.mode_follow_path()

        elif self.state == State.DID_A_FULL_ONE_EIGHTY:
            self.mode_180_turn()

        elif self.state == State.REWIND:
            self.mode_rewind()

        elif self.state == State.STOP:
            self.mode_stop()

        self.cmd_vel_pub.publish(self.twist)

    def mode_center_on_object(self):
        self.twist.linear.x = 0.2
        self.twist.angular.z = self.angle_blob

    def mode_wait_for_scan(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        rospy.sleep(5)
        self.state = State.FOLLOW_PATH
        # self.dont_look_for_objects_no_more = True

    def mode_follow_path(self):

        if self.obstacle_detected:
            print("Trying to back it up from the obstacle")
            self.twist.linear.x = -self.max_speed
            self.twist.angular.z = self.max_steering

        else:
            if(self.R_obstacleDetected and not self.L_obstacleDetected):
                self.twist.angular.z = self.max_steering
            elif(self.L_obstacleDetected and not self.R_obstacleDetected):
                self.twist.angular.z = -(self.max_steering)
            else:
                self.twist.angular.z = 0
            self.twist.linear.x = self.max_speed

    def mode_180_turn(self):
        self.twist.linear.x = self.max_speed
        self.twist.angular.z = self.max_steering
        if self.obstacle_detected:
            print("state: Rewinding")
            self.state = State.REWIND

    def mode_rewind(self):
        self.twist.linear.x = -self.max_speed
        self.twist.angular.z = - 1/16 * self.max_steering
        if abs(self.theta) > 0.99:
            print("state: Follow Path")
            self.state = State.FOLLOW_PATH

    def mode_stop(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0


def main():
    rospy.init_node('path_following')
    PathFollowing()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
