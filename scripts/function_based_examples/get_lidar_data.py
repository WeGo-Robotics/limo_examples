#! /usr/bin/env python

import rospy

import utils

import time

utils.startApplication()

while not rospy.is_shutdown():
    lidar_data = utils.getLidar()
    angle, center_range = utils.getLidarRangeUsingAngle(_lidar=lidar_data, _angle=0.0, _type="deg")
    print("{:.4f} degree, {:.4f} meter".format(angle, center_range))
rospy.loginfo("limo application end")

