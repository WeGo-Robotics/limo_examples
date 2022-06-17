#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ROS Publisher 및 Subscriber가 1개의 Node에 포함된 형태

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Vector3

rospy.init_node("ros_publisher_subscriber_basic")
TARGET_POSE_X = 6.0
# # ==========================================
# #               Class 없는 1번 형태
# # ==========================================
# # 
# # Subscriber에 Callback함수뿐만 아니라, Publisher도 함께 입력으로 전달하여
# # Callback함수 내부에서 Publisher를 사용할 수 있도록 한 형태
# # Subscribe하는 Topic의 속도 = Publish하는 Topic의 속도가 되는 형태
# # 데이터가 충분히 빠르게 전달이 될 수 있을 때 사용하는 것이 좋으며
# # Subscribe하는 Topic의 개수가 1개가 아닌, 그 이상의 개수가 될 경우 부적합한 형태일 수 있음
# #
# # ==========================================


# def callback(_data, _publisher):
#     if abs(_data.x - TARGET_POSE_X) < 0.1:
#         _publisher.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
#         rospy.loginfo("Goal Reached, Stop")
#         rospy.signal_shutdown("Goal Reached, Process Shutdown")
#     elif _data.x < TARGET_POSE_X:
#         _publisher.publish(Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0)))
#         rospy.loginfo("Go Forward to Goal")
#     elif _data.x > TARGET_POSE_X:
#         _publisher.publish(Twist(Vector3(-0.5, 0, 0), Vector3(0, 0, 0)))
#         rospy.loginfo("Go Backward to Goal")

# cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
# rospy.Subscriber("pose", Pose, callback, cmd_vel_pub)

# while not rospy.is_shutdown():
#     rospy.spin()

# # ================================================================
# #               Class를 사용한 2번 형태 (Timer 사용 X)
# # ================================================================
# # 
# # 하나의 Class로 묶어서 사용하므로, Callback함수에서도 Publisher 등의 모든 정보를 확인 및 사용할 수 있는 형태
# # 1번 형태와 동일하게 Subscribe하는 Topic의 속도 = Publish하는 Topic의 속도가 되는 형태
# # 데이터가 충분히 빠르게 전달이 될 수 있을 때 사용하는 것이 좋으며
# # Subscribe하는 Topic의 개수가 1개가 아닌, 그 이상의 개수가 될 경우에도 데이터를 Class 내부 변수로 저장만한다면
# # 사용하는데는 문제가 없으나, 아래 3번 형태에 비해서 형태가 좋지 못함
# # ================================================================
# class Turtle_Mover(object):
#     def __init__(self):
#         self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
#         rospy.Subscriber("pose", Pose, self.callback)
        
#     def callback(self, _data):
#         if abs(_data.x - TARGET_POSE_X) < 0.1:
#             self.cmd_vel_pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
#             rospy.loginfo("Goal Reached, Stop")
#             rospy.signal_shutdown("Goal Reached, Process Shutdown")

#         elif _data.x < TARGET_POSE_X:
#             self.cmd_vel_pub.publish(Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0)))
#             rospy.loginfo("Go Forward to Goal")
#         elif _data.x > TARGET_POSE_X:
#             self.cmd_vel_pub.publish(Twist(Vector3(-0.5, 0, 0), Vector3(0, 0, 0)))
#             rospy.loginfo("Go Backward to Goal")

# new_class = Turtle_Mover()
# while not rospy.is_shutdown():
#     rospy.spin()

# ================================================================
#               Class를 사용한 3번 형태 (Timer 사용 O)
# ================================================================
# 
# 하나의 Class로 묶어서 사용하므로, Callback함수에서도 Publisher 등의 모든 정보를 확인 및 사용할 수 있는 형태
# Timer를 사용하므로, 실제 처리 속도를 지정해서 사용할 수 있음
# Subscribe하는 Topic의 개수가 1개가 아닌, 그 이상의 개수가 될 경우에도 데이터를 Class 내부 변수로 저장만한다면
# 사용할 수 있으며, Timer를 통해 하나로 묶어서 처리가 가능한 형태
# ================================================================
class Turtle_Mover(object):
    def __init__(self):
        self.pose_init = False
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        rospy.Subscriber("pose", Pose, self.callback)
        rospy.Timer(rospy.Duration(1.0/30.0), self.timerCallback) # 30.0을 변경하여, 초당 처리 횟수 지정 가능
        
    def callback(self, _data): # 단순 데이터 저장용으로 사용, 실제 처리는 Timer Callback에서 진행
        self.pose_init = True
        self.pose_data = _data
   
    def timerCallback(self, _event): # event는 TimerCallback 함수에 필수적으로 필요하며, 사용할 필요 없음
        if self.pose_init == False:
            return

        if abs(self.pose_data.x - TARGET_POSE_X) < 0.1:
            self.cmd_vel_pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
            rospy.loginfo("Goal Reached, Stop")
            rospy.signal_shutdown("Goal Reached, Process Shutdown")

        elif self.pose_data.x < TARGET_POSE_X:
            self.cmd_vel_pub.publish(Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0)))
            rospy.loginfo("Go Forward to Goal")
        elif self.pose_data.x > TARGET_POSE_X:
            self.cmd_vel_pub.publish(Twist(Vector3(-0.5, 0, 0), Vector3(0, 0, 0)))
            rospy.loginfo("Go Backward to Goal")

new_class = Turtle_Mover()
while not rospy.is_shutdown():
    rospy.spin()
    