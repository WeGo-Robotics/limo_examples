#! /usr/bin/env python

import rospy

import utils

import time

utils.startApplication()

utils.goStraight(_linear_speed=0.2) # 0.2 m/s for linear_speed (move forward)
time.sleep(2) # wait until the move end
utils.goStraight(_linear_speed=-0.2) # -0.2 m/s for linear_speed (move backward)
time.sleep(2) # wait until the move end
utils.turn(_angular_speed=0.5, _unit="rad") # 0.5 rad/s for angular_speed (turn left)
time.sleep(2) # wait until the move end
utils.turn(_angular_speed=-0.5, _unit="rad") # -0.5 rad/s for angular_speed (turn right)
time.sleep(2)
utils.moveCmd(_linear_speed=0.2, _angular_speed=0.5, _unit="rad")
# 0.2 m/s for linear, 0.5 rad/s for angular, forward left
time.sleep(2)
utils.moveCmd(_linear_speed=0.2, _angular_speed=-0.5, _unit="rad")
# 0.2 m/s for linear, -0.5 rad/s for angular, forward right
time.sleep(2)
utils.moveCmd(_linear_speed=-0.2, _angular_speed=0.5, _unit="rad")
# -0.2 m/s for linear, 0.5 rad/s for angular, backward right
time.sleep(2)
utils.moveCmd(_linear_speed=-0.2, _angular_speed=-0.5, _unit="rad")
# 0.2 m/s for linear, -0.5 rad/s for angular, backward left
time.sleep(2)

rospy.loginfo("limo application end")

