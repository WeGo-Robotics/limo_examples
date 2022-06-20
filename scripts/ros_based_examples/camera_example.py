#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from limo_examples.cfg import camera_exampleConfig

import cv2

class CameraExample():
    def __init__(self):
        
        self.cvbridge = CvBridge()
        
        # ROS part
        rospy.init_node("camera_example")
        srv = Server(camera_exampleConfig, self.reconfigure_callback)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_raw_callback)
        self.img_pub = rospy.Publisher("canny", Image, queue_size=1)
        rospy.loginfo("Initialized, Node Spin Now")
        
# ==================================================
#                 Callback Functions
# ==================================================
    def reconfigure_callback(self, config, level):
        self.CANNY_MIN_TH = config.canny_min_threshold
        self.CANNY_MAX_TH = config.canny_max_threshold
        return config

    def image_raw_callback(self, data):
        
        frame = self.cvbridge.imgmsg_to_cv2(data,"bgr8")
        
        canny_result = cv2.Canny(frame, self.CANNY_MIN_TH, self.CANNY_MAX_TH)
        canny_msg = self.cvbridge.cv2_to_imgmsg(canny_result, "mono8")
        
        self.img_pub.publish(canny_msg)
        
def run():
    new_class = CameraExample()
    rospy.spin()
        
if __name__=="__main__":
    run()
