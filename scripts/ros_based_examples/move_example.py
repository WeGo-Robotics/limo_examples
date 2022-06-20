#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from dynamic_reconfigure.server import Server
from limo_examples.cfg import move_exampleConfig

class MoveExample():
    def __init__(self):
        # ROS part
        rospy.init_node("move_example")
        srv = Server(move_exampleConfig, self.reconfigure_callback)
        rospy.Subscriber("move_str",String,self.move_str_callback)
        self.twist_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        
    
# ==================================================
#                 Callback Functions
# ==================================================
    def reconfigure_callback(self, config, level):
        self.LINEAR_X = config.linear_x
        self.ANGULAR_Z = config.angular_z
        return config
    
    def move_str_callback(self, data):
        drive_data = Twist()
        
        if data.data == "forward":
            drive_data.linear.x = self.LINEAR_X
            print("LIMO forward")
            
            self.twist_pub.publish(drive_data)
            
        elif data.data == "rotate":
            drive_data.angular.z = self.ANGULAR_Z
            print("LIMO rotate")
            
            self.twist_pub.publish(drive_data)
            
        elif data.data == "stop":
            print("LIMO stop")
            
            self.twist_pub.publish(drive_data)
            
def run():
    new_class = MoveExample()
    rospy.loginfo_once("ROS Node Initialized")
    rospy.spin()
            
if __name__=="__main__":
    run()
