#! /usr/bin/env python

import rospy

import utils

import time

utils.startApplication()

while not rospy.is_shutdown():
    lidar_data = utils.getLidar()
    angle, center_range = utils.getLidarRangeUsingAngle(_lidar=lidar_data, _angle=0.0, _type="deg")
    if center_range >= 0.5 or center_range == 0.0:
        utils.goStraight(_linear_speed=0.2)
        rospy.loginfo("Center is clear, GO")
    else:
        utils.goStraight(_linear_speed=0.0) # means STOP
        rospy.loginfo("Blocked by Something, STOP")

rospy.loginfo("move_limo application end")

