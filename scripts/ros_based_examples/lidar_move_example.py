#! /usr/bin/env python 

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from limo_examples.cfg import lidar_move_exampleConfig

import math

class LidarMove():
    def __init__(self):
        
        self.flag = False
        
        # ROS part
        
        rospy.init_node("lidar_move_example")
        ## dynamic_reconfigure setup list
        srv = Server(lidar_move_exampleConfig, self.reconfigure_callback)
        self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        rospy.Subscriber("/scan",LaserScan,self.scan_callback)
    
# ==================================================
#                 Callback Functions
# ==================================================
    def reconfigure_callback(self, config, level):
        self.MIN_ANGLE_DEG = config.min_angle_deg
        self.MAX_ANGLE_DEG = config.max_angle_deg
        self.LINEAR_X = config.linear_x
        self.MIN_DIST = config.min_dist
        return config
    
    def scan_callback(self,data):
        for i,n in enumerate(data.ranges):
            angle = data.angle_min + data.angle_increment * i
            angle_deg = angle * 180 / math.pi
            
            x = n * math.cos(angle)
            y = n * math.sin(angle)
            
            if angle_deg < self.MAX_ANGLE_DEG and angle_deg > self.MIN_ANGLE_DEG and not n == 0:

                if x < self.MIN_DIST:
                    self.flag = True
                    self.distance = x
                    break
                          
        drive_data = Twist()
            
        if self.flag:
            print("distance is {}, LIMO stop".format(self.distance))
            
        else:
            drive_data.linear.x = self.LINEAR_X
            print("LIMO forward")
            
        self.pub.publish(drive_data)
        
def run():
    new_class = LidarMove()
    rospy.loginfo("Initialized, Node Spin Now")
    rospy.spin()
    
    
if __name__=="__main__":
    run()
