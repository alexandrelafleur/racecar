#!/usr/bin/env python

import rospy
import math 
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry

class State:
    FOLLOW_PATH = 0
    CENTER_ON_OBJECT = 1
    WAIT_FOR_SCAN = 2
    DID_A_FULL_ONE_EIGHTY = 3
    REWIND = 4

class PathFollowing:
    def __init__(self):
        self.twist = Twist()
        self.max_speed = rospy.get_param('~max_speed', 0.25)
        self.max_steering = rospy.get_param('~max_steering', 0.37)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber('obstacle_detected', Bool, self.obstacle_callback, queue_size=1)
        self.blob_sub = rospy.Subscriber('object_detected', Twist, self.blob_callback, queue_size=1)
        self.state_timer = rospy.Timer( rospy.Duration( 0.05 ), self.state_machine_callback )
        
        # VARIABLES
        self.goal_pos_x = 13.5
        self.end_pos_x = 0
        self.goal_pos_y = 2.1
        self.delta_pos = 0.5
        self.distance = 1.5
        self.dont_look_for_objects_no_more = False
        self.state = State.FOLLOW_PATH #
        self.obstacle_detected = False
        self.R_obstacleDetected = False
        self.L_obstacleDetected = False

    def scan_callback(self, msg):
        # Because the lidar is oriented backward on the racecar, 
        # if we want the middle value of the ranges to be forward:
        l2 = len(msg.ranges)/2;
        ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]
        
        # Obstacle front?   l2+l2/8
        self.R_obstacleDetected = False
        self.L_obstacleDetected = False
        for i in range(3*l2/4 - l2/16, 3*l2/4 + l2/16) :
            if np.isfinite(ranges[i]) and ranges[i]>0 and ranges[i] < self.distance:
                self.R_obstacleDetected = True
                break
        for i in range(5*l2/4 - l2/16, 5*l2/4 + l2/16) :
            if np.isfinite(ranges[i]) and ranges[i]>0 and ranges[i] < self.distance:
                self.L_obstacleDetected = True
                break

        self.state_machine_callback( None )
        
    def odom_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.theta = msg.pose.pose.orientation.z
        # print("orientation", self.theta)
        if abs(self.pos_x - self.goal_pos_x) < self.delta_pos : # and abs(self.pos_y - self.goal_pos_y) < self.delta_pos)
            if self.state == State.FOLLOW_PATH:
                if abs(self.theta) < 0.5:
                    print("DOING A 180")
                    self.state = State.DID_A_FULL_ONE_EIGHTY

    def blob_callback(self, msg):
        # print('Distance' , msg.linear.x)
        # print('Angle', msg.angular.z)
        print("got blob callback")
        self.angle_blob = msg.angular.z
        if(not self.dont_look_for_objects_no_more):
            if (msg.linear.x) < 2.0:
                self.state = State.WAIT_FOR_SCAN
                print('state: Waiting for scan')

            elif (msg.linear.x) < 2.5:
                self.state = State.CENTER_ON_OBJECT
                print('state: Trying to Center the object!')

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data
        if self.obstacle_detected:
            print("Obstacle :", msg.data)

    def state_machine_callback(self, state_timer):
        if self.state == State.CENTER_ON_OBJECT:
            self.mode_center_on_object()

        elif self.state == State.WAIT_FOR_SCAN:
            self.mode_wait_for_scan()

        elif self.state == State.FOLLOW_PATH:
            self.mode_follow_path()

        elif self.state == State.DID_A_FULL_ONE_EIGHTY:
            self.mode_180_turn()

        elif self.state == State.REWIND:
            self.rewind()

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
        if(self.R_obstacleDetected and not self.L_obstacleDetected):
            self.twist.angular.z = self.max_steering
        elif(self.L_obstacleDetected and not self.R_obstacleDetected):
            self.twist.angular.z = -(self.max_steering)
        else:
            self.twist.angular.z = 0
        self.twist.linear.x = self.max_speed

    def mode_180_turn(self):
        self.twist.linear.x = 0.2
        self.twist.angular.z = self.max_steering
        if self.obstacle_detected:
            print("state: Rewinding")
            self.state = State.REWIND

    def rewind(self):
        self.twist.linear.x = -0.2
        self.twist.angular.z = -self.max_steering
        if abs(self.theta) > 0.9:
            print("state: Follow Path")
            self.state = State.FOLLOW_PATH

def main():
    rospy.init_node('path_following')
    pathFollowing = PathFollowing()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

