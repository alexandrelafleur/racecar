#!/usr/bin/env python

import rospy
import math 
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool

class ObstacleDetector:
    def __init__(self):
        self.twist = Twist()
        self.max_speed = rospy.get_param('~max_speed', 1)
        self.max_steering = rospy.get_param('~max_steering', 0.37)
        self.distance = rospy.get_param('~distance', 0.75)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle_pub = rospy.Publisher('obstacle_detected', Bool, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)

    def scan_callback(self, msg):
    
        # Because the lidar is oriented backward on the racecar, 
        # if we want the middle value of the ranges to be forward:
        l2 = len(msg.ranges)/2;
        ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]
        
        # # Obstacle front?
        obstacleDetected = False
        for i in range(l2-l2/8, l2+l2/8) :
            if np.isfinite(ranges[i]) and ranges[i]>0 and ranges[i] < self.distance:
                obstacleDetected = True
                break
                
    #     self.twist.linear.x = 0
    #     self.twist.angular.z = -self.max_steering
        msg = Bool()
        msg.data = obstacleDetected
        self.obstacle_pub.publish(msg); # zero twist  
    #     rospy.loginfo("Obstacle detected! Stop!")      

def main():
    rospy.init_node('obstacle_detector')
    obstacleDetector = ObstacleDetector()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

