#!/usr/bin/env python

"""
Baxter RSDK Inverse Kinematics Example
"""
from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest,)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion,)
from std_msgs.msg import Header
import argparse
import baxter_interface
import numpy as np
import rospy
import struct
import sys
import time
import pprint as pp
from circle_detection.msg import detection_results, detection_results_array
import tf

class Baxter:

    def __init__(self):
        self.RorL = 'right'
        self.subscriberTopic = '/circledetection/' + self.RorL + '_circleArray'
        self.storage = rospy.Subscriber(self.subscriberTopic, detection_results_array, self.callback, callback_args=self.RorL, queue_size=1)
        self.listener = tf.TransformListener()
        self.robotEnable = baxter_interface.RobotEnable()
        self.robotEnable.enable()
        self.leftArm = baxter_interface.Limb('left')
        self.rightArm = baxter_interface.Limb('right')
        self.leftGripper = baxter_interface.Gripper('left')
        self.rightGripper = baxter_interface.Gripper('right')
        
    def __getArmPosition(self,limb):
        if limb=='left':
            position = self.leftArm.endpoint_pose()
        else:
            position = self.rightArm.endpoint_pose()
             
        x = position['position'].x
        y = position['position'].y
        z = position['position'].z
        return [x,y,z]

    def __getArmOrientation(self,limb):
        if limb=='left':
            orientation = self.leftArm.endpoint_pose()
        else:
            orientation = self.rightArm.endpoint_pose()

        x = orientation['orientation'].x
        y = orientation['orientation'].y
        z = orientation['orientation'].z
        w = orientation['orientation'].w
        return [x,y,z,w]

    def __inverseKinematics(self, limb, point, orientation):
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        rospy.wait_for_service(ns)
        # time.sleep(2)
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose = PoseStamped(
                    header=hdr,
                    pose=Pose(
                    position=Point(
                        x=point.x,
                        y=point.y,
                        z=point.z,
                    ),
                    orientation=Quaternion(
                        x=orientation[0],
                        y=orientation[1],
                        z=orientation[2],
                        w=orientation[3],
                    )
                )
            );
        ikreq.pose_stamp.append(pose)
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except rospy.ServiceException,e :
            rospy.loginfo("Service call failed: %s" % (e,))
        if (resp.isValid[0]):
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            return limb_joints
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
            return None
            
    def __moveArm(self,limb, point, orientation):
        angles = self.__inverseKinematics(limb, point, orientation)
        if not angles:
            return None

        if limb =='left':
            self.leftArm.move_to_joint_positions(angles) # 15 secs timeout default.
        else:
            self.rightArm.move_to_joint_positions(angles) # 15 secs timeout default.

    def closeLeftGripper(self):
        self.leftGripper.close()
        
    def openLeftGripper(self):
        self.leftGripper.open()

    def closeRightGripper(self):
        self.rightGripper.close()
        
    def openRightGripper(self):
        self.rightGripper.open()

    def callback(self, object_array,limb):
        storage = PoseStamped(pose=object_array.tracked_objects[0].pose, header=object_array.header)
        storage = self._transform(storage, 'base')
        storage = storage.pose.position
        storage.z+=0.15
        self.__moveArm(limb,storage,self.__getArmOrientation(limb))

        # lp_origin = self.__getArmPosition(limb)
        # lo_origin_or = self.__getArmOrientation(limb)
        # rp_origin = self.__getArmPosition(limb)
        # lo_origin_or = self.__getArmOrientation(limb)

    def _transform(self, msg, target_frame):
        if msg.header.frame_id != target_frame:
            try:
                t = self.listener.getLatestCommonTime(target_frame, msg.header.frame_id)
                msg.header.stamp = t
                return self.listener.transformPose(target_frame, msg)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
                rospy.logwarn(ex)
                return None
        else:
            return msg

def main():
    rospy.init_node("moving")
    zoidbot = Baxter()
    zoidbot.openLeftGripper()
    zoidbot.openRightGripper()
    rospy.spin()

if __name__ == '__main__':
    main()
