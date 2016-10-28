#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Example
"""
from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest,)
# from baxter_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion,)
# from organizer.srv import ImageSrv, ImageSrvResponse
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
        self.listener = tf.TransformListener()

    def enable(self):
        # An attribute of class RobotEnable to enable or disable Baxter.
        # It has the following methods:
        #   enable()    - enable all joints
        #   disable()   - disable all joints
        #   reset() - reset all joints, reset all jrcp faults, disable the robot
        #   stop()      - stop the robot, similar to hitting the e-stop button
        self.robotEnable = baxter_interface.RobotEnable()
        self.robotEnable.enable()
        self.leftArm = baxter_interface.Limb('left')
        self.rightArm = baxter_interface.Limb('right')
        # Waits for the image service of right hand camera to become available.
        # rospy.wait_for_service('last_image')
        # self.rightHandCamera = rospy.ServiceProxy('last_image', ImageSrv)
        self.leftGripper = baxter_interface.Gripper('left')
        
    def disable(self):
        self.robotEnable.disable()
        
    def getLeftArmPosition(self):
        position = self.leftArm.endpoint_pose()
        x = position['position'].x
        y = position['position'].y
        z = position['position'].z
        return [x,y,z]

    def getLeftArmOrientation(self):
        orientation = self.leftArm.endpoint_pose()
        x = orientation['orientation'].x
        y = orientation['orientation'].y
        z = orientation['orientation'].z
        w = orientation['orientation'].w
        return [x,y,z,w]
        
    def getRightArmPosition(self):
        position = self.rightArm.endpoint_pose()
        x = position['position'].x
        y = position['position'].y
        z = position['position'].z
        return [x,y,z]

    def getRightArmOrientation(self):
        orientation = self.rightArm.endpoint_pose()
        x = orientation['orientation'].x
        y = orientation['orientation'].y
        z = orientation['orientation'].z
        w = orientation['orientation'].w
        return [x,y,z,w]
        
    def inverseKinematics(self, limb, point, orientation):
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
        print "+++++++++++++++"
        # pp.pprint(pose)
        print"__________________"
        ikreq.pose_stamp.append(pose)
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
            # print resp
        except rospy.ServiceException,e :
            rospy.loginfo("Service call failed: %s" % (e,))
        if (resp.isValid[0]):
            #print("SUCCESS - Valid Joint Solution Found:")
            # Format solution into Limb API-compatible dictionary
            # limb_joints = dict(zip(resp.joints[0].names, resp.joints[0].angles))
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            return limb_joints
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
            return None
            
    # def ik(self, point, orientation):
    #     angles = self.inverseKinematics('left', point, orientation)
    #     if not angles:
    #         return False
    #     else:
    #         return True
            
    def moveArm(self,arm, point, orientation):
        angles = self.inverseKinematics(arm, point, orientation)
        if not angles:
            return None

        if arm =='left':
            self.leftArm.move_to_joint_positions(angles) # 15 secs timeout default.
        else:
        	self.rightArm.move_to_joint_positions(angles) # 15 secs timeout default.

    def closeLeftGripper(self):
        self.leftGripper.close()
        
    def openLeftGripper(self):
        self.leftGripper.open()

    def run(self):
        arm='right'
        if arm=='left':
            origin = self.getLeftArmPosition()
            origin_or = self.getLeftArmOrientation()
        else:
            origin = self.getRightArmPosition()
            origin_or = self.getRightArmOrientation()

        # origin[2]+=0.1
        print self.moveArm(arm,origin,origin_or)

    def subs(self):
        subscriberTopic = '/circledetection/right_circleArray'
        self.storage = rospy.Subscriber(subscriberTopic, detection_results_array, self.callback, queue_size=1)

    def callback(self, object_array):
        # print object_array.tracked_objects[0]

        storage = PoseStamped(pose=object_array.tracked_objects[0].pose, header=object_array.header)
        storage = self._transform(storage, 'base')
        storage = storage.pose.position
        storage.z+=0.15
        print storage
        print self.getRightArmPosition()
        self.moveArm('right',storage,self.getRightArmOrientation())


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
    zoidbot.enable()
    # zoidbot.openLeftGripper()
    # zoidbot.closeLeftGripper()
    # zoidbot.run()
    zoidbot.subs()
    rospy.spin()

if __name__ == '__main__':
    main()
