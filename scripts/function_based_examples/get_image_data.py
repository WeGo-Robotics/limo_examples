#! /usr/bin/env python

import rospy

import utils

import time

utils.startApplication()

while not rospy.is_shutdown():
    image_data = utils.getImage(_type="opencv")
    utils.showOpenCvImage(image_data)

rospy.loginfo("limo application end")

