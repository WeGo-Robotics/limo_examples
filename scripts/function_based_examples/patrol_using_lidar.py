#! /usr/bin/env python

import rospy

import utils

import time
import random

utils.startApplication()

angles = [-60.0, 0.0, 60.0]

while not rospy.is_shutdown():
    center_ranges = []
    lidar_data = utils.getLidar()
    for angle in angles:
        angle, center_range = utils.getLidarRangeUsingAngle(_lidar=lidar_data, _angle=angle, _type="deg")
        center_ranges.append(center_range)
    
    min_idx = 0
    min_val = 100.0
    for i, range in enumerate(center_ranges):
        if range < min_val:
            min_idx = i
            min_val = range
    
    if min_val == 100.0:
        utils.goStraight(_linear_speed=0.2)
        rospy.loginfo("Clear, GO")
    elif min_val < 0.5 and not min_val == 0.0:
        utils.goStraight(_linear_speed=0.0) # means STOP
        rospy.loginfo("Blocked by Something, STOP")
        time.sleep(3)
        if min_idx == 0:
            # utils.moveCmd(_linear_speed=0.1, _angular_speed=1.0)
            move_command = "LEFT"
            rospy.loginfo("Right is Blocked, TURN Left")
        elif min_idx == 1:
            # utils.moveCmd(_linear_speed=0.1, _angular_speed=2.0)
            move_command = "TURN"
            rospy.loginfo("Center is Blocked, TURN")
        elif min_idx == 2:
            # utils.moveCmd(_linear_speed=0.1, _angular_speed=-1.0)
            move_command = "RIGHT"
            rospy.loginfo("Left is Blocked, TURN Right")
        start_time = rospy.Time.now().to_sec()
        if random.choice([True, False]):
            tmp_speed = 0.7 + random.random() * 0.5
        else:
            tmp_speed = -0.7 - random.random() * 0.5
                
        while rospy.Time.now().to_sec() - start_time <= 3.0:
            if move_command == "LEFT":
                utils.moveCmd(_linear_speed=0.0, _angular_speed=.7)
            elif move_command == "RIGHT":
                utils.moveCmd(_linear_speed=0.0, _angular_speed=-.7)
            elif move_command == "TURN":
                    
                utils.moveCmd(_linear_speed=0.0, _angular_speed=tmp_speed)
                
    else:
        utils.goStraight(_linear_speed=0.2)
        rospy.loginfo("Clear, GO")

rospy.loginfo("move_limo application end")

