#!/usr/bin/env python

import rospy
from tf import TransformListener
import geometry_msgs.msg


class TransformS(object):
    
    def __init__(self):
        rospy.init_node('whycon_transform')
        self.tf = TransformListener()
        print "Starting tranformer"
        rospy.Subscriber("/whycon_right/poses",geometry_msgs.msg.PoseArray, self.handle_whycon)
        self.stick_pose_pub = rospy.Publisher("/grab", geometry_msgs.msg.PoseStamped, queue_size=1)
    
    def handle_whycon(self, msg):
        for i in msg.poses:
            stick_pose_or= geometry_msgs.msg.PoseStamped()
            stick_pose_or.header=msg.header
            stick_pose_or.pose=i

            self.tf.waitForTransform(stick_pose_or.header.frame_id, "/base",stick_pose_or.header.stamp, rospy.Duration(0.5))
            #(trans,rot) = self.tf.lookupTransform('/base', stick_pose_or.header.frame_id, rospy.Time(0))
            stick_pose_d= self.tf.transformPose('/base', stick_pose_or)
            stick_pose_d.pose.position.z= stick_pose_d.pose.position.z+0.02
            #stick_pose_d.pose = trans
            stick_pose_d.pose.orientation.x= 0.0
            stick_pose_d.pose.orientation.y= 1.0
            stick_pose_d.pose.orientation.z= 0.0
            stick_pose_d.pose.orientation.w= 0.0
            self.stick_pose_pub.publish(stick_pose_d)


if __name__ == '__main__':
    teleop = TransformS()
    rospy.spin()