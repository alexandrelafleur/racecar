#!/usr/bin/env python3

import rospy
import math
import subprocess 
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, CompressedImage
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from labo_brushfire import get_path
import cv2
from libbehaviors import *
from nav_msgs.srv import GetMap
from libbehaviors import occupancy_grid_to_maze, brushfire
from astar import pfind_new

class State:
    FOLLOW_PATH = 0
    CENTER_ON_OBJECT = 1
    WAIT_FOR_SCAN = 2
    DID_A_FULL_ONE_EIGHTY = 3
    REWIND = 4
    STOP = 5

class Blob:
    def __init__(self, x, y, id):
        self.id = id
        self.abs_pos_x = x
        self.abs_pos_y = y

class PathFollowing:
    def __init__(self):
        print("initializing node")
        self.twist = Twist()
        self.max_speed = rospy.get_param('~max_speed', 0.25)
        self.max_steering = rospy.get_param('~max_steering', 0.37)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber('obstacle_detected', Bool, self.obstacle_callback, queue_size=1)
        self.blob_sub = rospy.Subscriber('object_detected', Twist, self.blob_callback, queue_size=1)
    
        self.blob_sub = rospy.Subscriber('/racecar/raspicam_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)
        self.state_timer = rospy.Timer( rospy.Duration( 0.05 ), self.state_machine_callback )
        
        # VARIABLES
        self.start_pos_x = 0
        self.start_pos_y = 0
        self.goal_pos_x = 11.8
        self.end_pos_x = 0
        self.goal_pos_y = 2.2
        self.delta_pos = 0.5
        self.blob_delta_pos = 3
        self.distance = 1.5
        self.dont_look_for_objects_no_more = False
        self.state = State.FOLLOW_PATH #
        self.obstacle_detected = False
        self.R_obstacleDetected = False
        self.L_obstacleDetected = False
        self.reached_the_end = False
        self.visited_blobs = []
        self.last_blob = None
        self.picture_buffer = b''

    def scan_callback(self, msg):
        # Because the lidar is oriented backward on the racecar, 
        # if we want the middle value of the ranges to be forward:
        l2 = int(len(msg.ranges)/2)
        ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]
        
        # Obstacle front?   l2+l2/8
        self.R_obstacleDetected = False
        self.L_obstacleDetected = False
        angle_offset = l2/8
        start_angle = int(3*l2/4 - angle_offset)
        stop_angle = int(3*l2/4 + angle_offset)
        closest_right = 10000
        for i in range(start_angle, stop_angle) :
            if (np.isfinite(ranges[i]) and ranges[i]>0 and ranges[i] < self.distance):
                closest_right = min(closest_right, ranges[i])
                self.R_obstacleDetected = True
                break
        start_angle = int(5*l2/4 - angle_offset)
        stop_angle = int(5*l2/4 + angle_offset)
        closest_left = 10000
        for i in range(start_angle, stop_angle) :
            if (np.isfinite(ranges[i]) and ranges[i]>0 and ranges[i] < self.distance):
                closest_left = min(closest_left, ranges[i])
                self.L_obstacleDetected = True
                break

        if(self.R_obstacleDetected and not self.L_obstacleDetected):
            pass
            # print("obstacle on the right")
        elif(self.L_obstacleDetected and not self.R_obstacleDetected):
            pass
            # print("obstacle on the left")
        elif(self.R_obstacleDetected and self.L_obstacleDetected):
            if(closest_right < closest_left):
                # print("turn left, distances(l,r):", closest_left, closest_right)
                self.L_obstacleDetected = False
                self.R_obstacleDetected = True
            elif(closest_left < closest_right):
                # print("turn right, distances(l,r):", closest_left, closest_right)
                self.L_obstacleDetected = True
                self.R_obstacleDetected = False

        self.state_machine_callback( None )
        
    def odom_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.theta = msg.pose.pose.orientation.z*3.14
        # print("pos_x", self.pos_x)
        if (abs(self.pos_x - self.goal_pos_x) < self.delta_pos) : # and abs(self.pos_y - self.goal_pos_y) < self.delta_pos)
            if self.state == State.FOLLOW_PATH:
                if abs(self.theta) < 0.5:
                    print("DOING A 180")
                    self.state = State.DID_A_FULL_ONE_EIGHTY
                    self.reached_the_end = True

        if (abs(self.pos_x - self.start_pos_x) < self.delta_pos) : # and abs(self.pos_y - self.goal_pos_y) < self.delta_pos)
            if self.state == State.FOLLOW_PATH and self.reached_the_end:
                print("REACHED THE START")
                self.state = State.STOP

    def blob_callback(self, msg):
        print("found from the callback object")

        self.angle_blob = msg.angular.z
        self.distance_blob = msg.linear.x

        if (self.state == State.CENTER_ON_OBJECT and self.distance_blob < 2.0):
            self.state = State.WAIT_FOR_SCAN
            self.take_obstacle_picture(self.last_blob)
            get_path(self.last_blob.abs_pos_x, self.last_blob.abs_pos_y, self.last_blob.id)
            print('state: Waiting for scan')

        elif self.distance_blob < 2.5:
            
            absolute_blob_pos_x = self.pos_x + self.distance_blob*math.cos(self.angle_blob + self.theta)
            absolute_blob_pos_y = self.pos_y + self.distance_blob*math.sin(self.angle_blob + self.theta)

            # check if seen before
            seenBefore = False
            for blob in self.visited_blobs:
                if abs(absolute_blob_pos_x - blob.abs_pos_x) < self.blob_delta_pos:
                    if abs(absolute_blob_pos_y - blob.abs_pos_y) < self.blob_delta_pos:
                        seenBefore = True
            if not seenBefore:
                id = len(self.visited_blobs)        
                blob = Blob(absolute_blob_pos_x, absolute_blob_pos_y, id)
                self.last_blob = blob
                print("Calling find_path()")
                self.visited_blobs.append(blob)
                blob_nb = len(self.visited_blobs)
                self.state = State.CENTER_ON_OBJECT

                print('state: Trying to Center the object!')        

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data
        if self.obstacle_detected:
            print("Obstacle :", msg.data)
            

    def image_callback(self, msg):
        self.picture_buffer = msg.data


    def take_obstacle_picture(self, blob):
        print("TAKING A PICTURE")
        print("Putting da pictcha in da file")
        file1 = open("/home/racecar/catkin_ws/data/photo_debris_" + str(blob.id) + ".png","w+b")
        file1.write(self.picture_buffer)
        file1.close()
        # print("Putting it in da file")
        file1 = open("/home/racecar/catkin_ws/data/Report.txt","a")
        file1.write("debris_nb_" + str(blob.id) + "\n")
        file1.write("X :" + str(blob.abs_pos_x) + " Y : " + str(blob.abs_pos_y) + "\n")
        file1.write("photo_debris_" + str(blob.id) + "\n")
        file1.write("trajectory_debris_" + str(blob.id) + "\n\n")
        file1.close()

    def state_machine_callback(self, state_timer):
        print("state:", self.state)
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
        self.twist.linear.x = 0.8
        self.twist.angular.z = self.angle_blob
        print("centering on object")

    def mode_wait_for_scan(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        print("wating for scan")
        rospy.sleep(5)
        self.state = State.FOLLOW_PATH
        # self.dont_look_for_objects_no_more = True

    def mode_follow_path(self):

        if self.obstacle_detected:
            print("Back it up Terry!")
            self.twist.linear.x = -self.max_speed
            self.twist.angular.z = self.max_steering

        else:
            if(self.R_obstacleDetected and not self.L_obstacleDetected):
                self.twist.angular.z = self.max_steering
            elif(self.L_obstacleDetected and not self.R_obstacleDetected):
                self.twist.angular.z = -self.max_steering
            else:
                self.twist.angular.z = 0
            self.twist.linear.x = self.max_speed

    def mode_180_turn(self):
        self.twist.linear.x = self.max_speed
        self.twist.angular.z = self.max_steering
        print("7")
        if self.obstacle_detected:
            print("state: Rewinding")
            self.state = State.REWIND

    def mode_rewind(self):
        self.twist.linear.x = -self.max_speed
        self.twist.angular.z = - 1/16 * self.max_steering
        print("rewinding")
        if abs(self.theta) > 0.99:
            print("state: Follow Path")
            self.state = State.FOLLOW_PATH

    def mode_stop(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        print("stopping")

def main():
    rospy.init_node('path_following')
    pathFollowing = PathFollowing()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

