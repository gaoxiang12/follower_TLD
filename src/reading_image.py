#! /usr/bin/python
import roslib
roslib.load_manifest('slam_gx')
import sys
import rospy
import cv2
import numpy as np
import pdb
from matplotlib import pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class image_converter():
    def __init__(self):
        cv2.namedWindow('Image color')
        cv2.namedWindow('Image depth')
        self.bridge = CvBridge()
        self.image_sub_color = rospy.Subscriber('/camera/rgb/image_color',Image, self.callback_color)
        self.image_sub_depth = rospy.Subscriber('/camera/depth_registered/image_raw', Image, self.callback_depth)
        self.depth_num = 1

    def callback_color(self, data):
        try:
            cv_image = np.asarray(self.bridge.imgmsg_to_cv(data, 'passthrough'))
        except CvBridgeError, e:
            print e
        cv2.imshow('Image color', cv_image)
        cv2.waitKey(1)
    def callback_depth(self, data):
        try:
            cv_image = np.asarray(self.bridge.imgmsg_to_cv(data, 'passthrough'))
        except CvBridgeError, e:
            print e

        minval, maxval, minLoc, maxLoc = cv2.minMaxLoc(cv_image)
        print minval, maxval
        if maxval > 0:
            cv_image[:] = cv_image[:]/maxval*255
        cv2.imshow('Image depth', cv_image)
        a = (0xff & cv2.waitKey(1))
        if a == ord('d'):
            cv2.imwrite('depth_'+str(self.depth_num)+'.jpg', cv_image)
            self.depth_num += 1
            print 'depth image saved'
    
if __name__=='__main__':
    ic = image_converter()
    rospy.init_node('image_reader', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"
    cv2.destroyAllWindows()
