#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from my_pkg.msg import two_ints

import os
import numpy as np
import tf
import math
import sys

# PID CONTROL PARAMS
kp = 1.115
kd = 0.08
ki = 0.0000015

# kp = 0.42
# kd = 0.01
# ki = 0.0005


L = 0.4

# L = 1

servo_offset = 0.0
prev_error = 0.0
error = 0.0
integral = 0.0


class NewcastleDrive(object):

    def __init__(self):

        self.index = 0

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber('/odom', Odometry, self.pose_callback, queue_size=1)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        self.pub = rospy.Publisher('/two_ints', two_ints, queue_size=1)

    def select_velocity(self, angle):
        if abs(angle) <= 5 * math.pi / 180:
            velocity = 4.5
        elif abs(angle) <= 10 * math.pi / 180:
            velocity = 4.0
        elif abs(angle) <= 15 * math.pi / 180:
            velocity = 4.0
        elif abs(angle) <= 20 * math.pi / 180:
            velocity = 4.0
        else:
            velocity = 3.0
        return velocity

    def findangle(self, data):
        lid = []
        maxindex = 0
        i = 0
        x = 0
        readingold = 0
        while i < len(data.ranges):
            if (i <= 180) or (i >= 900):
                x = 0
                reading = 0
            elif data.ranges[i] <= 5:
                x = 0
                reading = 0


            else:
                x += 1
                reading = reading + data.ranges[i] - 0.001 * abs(540 - i)
                if x > 20 and reading > readingold:
                    readingold = reading
                    maxindex = i - x/2
            i += 1
        # print(len(lid))
        return maxindex

    def driver(self, angle, velocity):

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "drive"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def scan_callback(self, data):

        self.lidar = data

        # publish drive message

    def pose_callback(self, pose_msg):

        quaternion = np.array([pose_msg.pose.pose.orientation.x,
                               pose_msg.pose.pose.orientation.y,
                               pose_msg.pose.pose.orientation.z,
                               pose_msg.pose.pose.orientation.w])

        self.euler = tf.transformations.euler_from_quaternion(quaternion)
        self.position = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y]

        velocity = 0.0
        angle = 0.0

        # ---------------------------------- process lidar and find destination points

        anglefound = self.findangle(self.lidar)
        angle_to_dist = (135 - anglefound / 4)

        # prev_angle = 0

        # if (angle_to_dist)

        global integral
        global prev_error
        global kp
        global ki
        global kd

        # servo_offset = 0.4 * 1.0 / 2.0

        pid_angle = -kp * angle_to_dist  # + ki * (integral + angle_to_dist * servo_offset) + kd * (angle_to_dist - prev_error) / servo_offset

        # prev_error = angle_to_dist

        if (angle_to_dist > 40) or (angle_to_dist < -40):
            pid_angle = np.clip(pid_angle, -0.4, 0.4)
        else:
            pid_angle /= 100

        # print(angle_to_dist, pid_angle)

        # ---------------------------------------

        # destination = [1, 0]

        # compute angle to the WAYPOINT (destination)
        # destination_to_point = math.sqrt((abs(self.position[0] - destination[0]) ** 2) + (abs(self.position[1] - destination[1]) ** 2))
        # l2_0 = [destination[0] - self.position[0], destination[1] - self.position[1]]
        # goaly_veh = -math.sin(self.euler[2]) * l2_0[0] + math.cos(self.euler[2]) * l2_0[1]
        # arc = 2 * goaly_veh / (destination_to_point ** 2)
        # angle = 0.3 * arc
        # angle = np.clip(angle, -0.35, 0.35)
        # ----------------------------------------------------------

        # get lidar data with self.data from scan_callback
        self.driver(pid_angle, self.select_velocity(pid_angle))


if __name__ == '__main__':
    rospy.init_node('newcastle_drive_node')
    nd = NewcastleDrive()
    rospy.spin()
