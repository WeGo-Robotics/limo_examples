#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, CompressedImage
from cv_bridge import CvBridge

import cv2

from math import pi
import time

def startApplication():
    rospy.init_node("limo_examples")
    rospy.loginfo("limo application start")
    topic_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=5)
    time.sleep(2)

def showOpenCvImage(_image, _window_name="myImage"):
    cv2.imshow(_window_name, _image)
    cv2.waitKey(1)
    
def getLidarRangeUsingAngle(_lidar, _angle=0.0, _type="deg"):
    '''
        Get Closest Angle's Range in meter (LIMO's LEFT --> 90 degree, RIGHT --> -90 degree)
        _type should be "rad" or "deg"
        return (angle, range) --> degree & meter
    '''
    if _type == "rad":
        _angle = _angle * 180 / pi
    elif _type == "deg":
        _angle = _angle
    else:
        rospy.logwarn('_type should be "rad" or "deg"')
    
    if _angle < -90.0:
        rospy.logwarn('_angle should greater than -90.0')
    
    elif _angle > 90.0:
        rospy.logwarn('_angle should less than 90.0')
    
    angle_deg = [(_lidar.angle_min + i * _lidar.angle_increment) * 180 /pi for i, _ in enumerate(_lidar.ranges)]
    for i, angle in enumerate(angle_deg):
        if _angle - 1.5 < angle < _angle + 1.5 and not _lidar.ranges[i] < 0.01:
            return (angle, _lidar.ranges[i])
        
    return (_angle, 0.0)
    

def getRosTopicData(_topic_name, _msg_type):
    '''
        Get Data from ROS Topic One Time --> need topic name & msg type
    '''
    try:
        return rospy.wait_for_message(_topic_name, _msg_type, 3.0)
    except Exception as e:
        rospy.logwarn("Cannot get the {} data, please check the topic and type, return None".format(_topic_name))
        return None
    
def getLidar():
    '''
        Get Lidar Data from /scan Topic One Time
    '''
    return getRosTopicData("scan", LaserScan)

def getImage(_type="opencv"):
    '''
        Get Image Data from /camera/rgb/image_raw/compressed Topic One Time
    '''
    if _type == "opencv":
        bridge = CvBridge()
        return bridge.compressed_imgmsg_to_cv2(getRosTopicData("/camera/rgb/image_raw/compressed", CompressedImage))
    elif _type == "ROS":
        return getRosTopicData("/camera/rgb/image_raw/compressed", CompressedImage)
    else:
        rospy.logwarn('type should be "opencv" or "ROS", default is "opencv"')

def goStraight(_linear_speed=0.2):
    '''
        Move Straight --> _linear_speed (m/s)
        return True if Success
        return False is Fail
    '''
    MAX_SPEED = 2.0
    MIN_SPEED = -2.0
    try:
        topic_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        drive_data = Twist()
        drive_data.linear.x = max(min(_linear_speed, MAX_SPEED), MIN_SPEED)
        topic_publisher.publish(drive_data)
        time.sleep(0.1)
        return True
    except Exception as e:
        rospy.logwarn(e)
        return False
    
def turn(_angular_speed=0.5, _unit="rad"):
    '''
        Turn --> _angular_speed (rad/s) 
        + for turn counter-clockwise, - for turn clockwise
        return True if Success
        return False is Fail
    '''
    if _unit == "deg":
        _angular_speed = _angular_speed * pi / 180
    elif _unit == "rad":
        _angular_speed = _angular_speed
    else:
        rospy.logwarn('unit should be "deg" or "rad", default is "rad"')
        return False
    
    MAX_SPEED = 2.0
    MIN_SPEED = -2.0
    try:
        topic_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        drive_data = Twist()
        drive_data.angular.z = max(min(_angular_speed, MAX_SPEED), MIN_SPEED)
        topic_publisher.publish(drive_data)
        time.sleep(0.1)
        return True
    except Exception as e:
        rospy.logwarn(e)
        return False    
    
def moveCmd(_linear_speed=0.2, _angular_speed=0.5, _unit="rad"):
    '''
        Merge the function goStraight & turn
        Move Straight --> _linear_speed (m/s)
        Turn --> _angular_speed (rad/s)
        + for turn counter-clockwise, - for turn clockwise
        return True if Success
        return False is Fail
    '''
    if _unit == "deg":
        _angular_speed = _angular_speed * pi / 180
    elif _unit == "rad":
        _angular_speed = _angular_speed
    else:
        rospy.logwarn('unit should be "deg" or "rad"')
        return False
    
    MAX_SPEED = 2.0
    MIN_SPEED = -2.0
    try:
        topic_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        drive_data = Twist()
        drive_data.linear.x = max(min(_linear_speed, MAX_SPEED), MIN_SPEED)
        drive_data.angular.z = max(min(_angular_speed, MAX_SPEED), MIN_SPEED)
        topic_publisher.publish(drive_data)
        time.sleep(0.1)
        return True
    except Exception as e:
        rospy.logwarn(e)
        return False
