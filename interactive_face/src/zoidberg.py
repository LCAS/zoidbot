#!/usr/bin/python
import cv2
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import numpy as np
import rosparam
import rospy
from sensor_msgs.msg import Image
import sys
import tf
import rospkg

class interactive_face:

    def __init__(self):
        self.pub = rospy.Publisher('/Baxter/Face/Zoidberg', Image, queue_size=10)
        self.bridge = CvBridge()
        self.cv_bg = np.zeros((600,1024,3), np.uint8)
        self.cv_bg[:,:] = (91,91,240)
        self.rospack = rospkg.RosPack()
        self.cv_blink = []
        for i in range(0,4):
            self.cv_blink.append(cv2.imread(self.rospack.get_path('interactive_face')+'/assets/face_blink' + str(i) + '.png',cv2.CV_LOAD_IMAGE_COLOR))
        self.cv_face = self.cv_blink[0]
        self.face_offsetX = 370
        self.face_offsetY = 150
        rospy.Timer(rospy.Duration(3), self.blink)

    def run(self):
        l_img = self.cv_bg
        self.rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            s_img = self.cv_face
            l_img[self.face_offsetY:(s_img.shape[0]+self.face_offsetY),self.face_offsetX:(s_img.shape[1]+self.face_offsetX),:] = s_img
            self.pub.publish(self.bridge.cv2_to_imgmsg(l_img))
            self.rate.sleep()
        
    def blink(self,event):
        r = rospy.Rate(25)
        r2 = rospy.Rate(15)
        for i in range(0,4):
            self.cv_face = self.cv_blink[i]
            r.sleep()
        r.sleep()
        for i in reversed(range(0,4)):
            self.cv_face = self.cv_blink[i]
            r2.sleep()

def main():
        rospy.init_node('interactive_face', anonymous=True)
        in_face = interactive_face()
        in_face.run();

if __name__ == '__main__':
    main()