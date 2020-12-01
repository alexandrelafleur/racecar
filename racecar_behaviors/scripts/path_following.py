#!/usr/bin/env python

import rospy
import math 
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class PathFollowing:
    def __init__(self):
        self.twist = Twist()
        self.max_speed = rospy.get_param('~max_speed', 1)
        self.max_steering = rospy.get_param('~max_steering', 0.37)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        self.blob_sub = rospy.Subscriber('object_detected', String, self.blob_callback, queue_size=1)
        self.distance = 1.5;

    def scan_callback(self, msg):
        # Because the lidar is oriented backward on the racecar, 
        # if we want the middle value of the ranges to be forward:
        l2 = len(msg.ranges)/2;
        ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]
        
        # Obstacle front?   l2+l2/8
        R_obstacleDetected = False
        L_obstacleDetected = False
        for i in range(3*l2/4 - l2/16, 3*l2/4 + l2/16) :
            if np.isfinite(ranges[i]) and ranges[i]>0 and ranges[i] < self.distance:
                R_obstacleDetected = True
                print("obst detected right")
                break
        for i in range(5*l2/4 - l2/16, 5*l2/4 + l2/16) :
            if np.isfinite(ranges[i]) and ranges[i]>0 and ranges[i] < self.distance:
                L_obstacleDetected = True
                print("obst detected left")
                break

        if(R_obstacleDetected and not L_obstacleDetected):
            self.twist.angular.z = self.max_steering
        elif(L_obstacleDetected and not R_obstacleDetected):
            self.twist.angular.z = -(self.max_steering)
        else:
            self.twist.angular.z = 0

        self.twist.linear.x = self.max_speed           
        self.cmd_vel_pub.publish(self.twist)
        
    def odom_callback(self, msg):
        rospy.loginfo("Current speed = %f m/s", msg.twist.twist.linear.x)

    def blob_callback(self, msg):
        pass


def main():
    rospy.init_node('path_following')
    pathFollowing = PathFollowing()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

