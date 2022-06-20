#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.server import Server
from limo_examples.cfg import lidar_exampleConfig

import math

class LidarExample():
    def __init__(self):
        # ROS part
        
        rospy.init_node("lidar_example")
        ## dynamic_reconfigure setup list
        srv = Server(lidar_exampleConfig, self.reconfigure_callback)
        rospy.Subscriber("scan",LaserScan,self.scan_callback)

# ==================================================
#                 Callback Functions
# ==================================================
    def reconfigure_callback(self, config, level):
        self.MIN_ANGLE_DEG = config.min_angle_deg
        self.MAX_ANGLE_DEG = config.max_angle_deg
        return config

    def scan_callback(self, data):
        for i,n in enumerate(data.ranges):
            angle = data.angle_min + data.angle_increment * i
            angle_deg = angle * 180 / math.pi
            
            x = n * math.cos(angle)
            y = n * math.sin(angle)
            
            if angle_deg < self.MAX_ANGLE_DEG and angle_deg > self.MIN_ANGLE_DEG and not n == 0:
                print("lidar angle and range : ({},{})".format(angle, n))
                print("lidar x and y : ({},{})".format(x, y))

def run():
    new_class = LidarExample()
    rospy.loginfo_once("ROS Node Initialized")
    rospy.spin()
            
if __name__=="__main__":
    run()
