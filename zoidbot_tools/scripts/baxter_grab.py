#!/usr/bin/env python

"""
Baxter RSDK Inverse Kinematics Example
"""

#from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion,)

#import argparse

#import numpy as np

#import struct
import sys
#import time
#import pprint as pp
#from circle_detection.msg import detection_results, detection_results_array
#import tf
import rospy

from std_msgs.msg import Header
import geometry_msgs.msg
import moveit_commander
#import moveit_msgs.msg
import zoidbot_tools.srv
#strands_navigation_msgs.srv
from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest,)
import baxter_interface
from baxter_interface import CHECK_VERSION

class BaxterGrab:

    def __init__(self):
        self.RorL = 'right'
        #self.subscriberTopic = '/circledetection/' + self.RorL + '_circleArray'
        #self.storage = rospy.Subscriber(self.subscriberTopic, detection_results_array, self.callback, callback_args=self.RorL, queue_size=1)
        #self.listener = tf.TransformListener()
        self.pick_block_srv=rospy.Service('/pick_block_right', zoidbot_tools.srv.PickBlock, self.pick_block_cb)
        #This service switches topological map
         
        self.target_pose_pub = rospy.Publisher("/grabbing", geometry_msgs.msg.PoseStamped, queue_size=1,latch=True)
        self.robotEnable = baxter_interface.RobotEnable()
        self.robotEnable.enable()
        self.leftArm = baxter_interface.Limb('left')
        self.rightArm = baxter_interface.Limb('right')
        
        self._gripper_left = baxter_interface.Gripper("left", CHECK_VERSION)
        self._gripper_right = baxter_interface.Gripper("right", CHECK_VERSION)
        #self.setup_moveit_commander()

        # Verify Grippers Have No Errors and are Calibrated
        self._gripper_left.reset()
        self._gripper_right.reset()
        self._gripper_left.calibrate()
        self._gripper_right.calibrate()
            

    def pick_block_cb(self, req):
        self._gripper_right.open()
        rospy.sleep(0.5)
        self._gripper_left.open()
        if self.plan_to(0):
            self._gripper_right.open()
            rospy.sleep(0.5)
            self._gripper_left.open()
        else:
            self.plan_to(3)
            return False
        if self.plan_to(1):
            self._gripper_left.close()
            rospy.sleep(0.5)
            self._gripper_right.close()
        else:
            self.plan_to(3)
            return False
        if  self.plan_to(3):
            #self._gripper_right.open()
            rospy.sleep(1)
            #self._gripper_left.open()
        else:
            return False
        if  self.plan_to(2):
            self._gripper_right.open()
            rospy.sleep(1)
            self._gripper_left.open()
        else:
            self.plan_to(3)
            return False
        if  self.plan_to(3):
            self._gripper_right.open()
            rospy.sleep(0.5)
            self._gripper_left.open()
        else:
            return False
        return True

    def plan_to(self, step):
        if step==0:
            pose_target = geometry_msgs.msg.Pose()
            print "Waiting for whycon"
            try:
                self.last_whyconpose=rospy.wait_for_message("/grab",geometry_msgs.msg.PoseStamped, timeout=90)
            except(rospy.ROSException), e:
                print "No objects, aborting..."
                print "Error message: ", e
                return False

            print self.last_whyconpose
            pose_target = self.last_whyconpose.pose
            pose_target.position.x = pose_target.position.x -0.02
            pose_target.position.z = pose_target.position.z +0.11
    #        pose_target.position.x = 0.769808727835
    #        pose_target.position.y = -0.259731727372
    #        pose_target.position.z = -0.103158764323
            pose_target.orientation.x = -0.154522394097
            pose_target.orientation.y = 0.983299780141
            pose_target.orientation.z = 0.0295583007957
            pose_target.orientation.w = 0.0914914146322
            rospy.sleep(0.5)
            pos_tar_st = geometry_msgs.msg.PoseStamped()
            pos_tar_st.header.stamp = rospy.Time.now()
            pos_tar_st.header.frame_id = 'base'
            pos_tar_st.pose = pose_target
            self.target_pose_pub.publish(pos_tar_st)
            rospy.sleep(0.5)
            print "Planning_to"
            print pose_target
            target_joints=self.__inverseKinematics('right',pose_target.position, pose_target.orientation)
            print target_joints
            if target_joints:
                self.move_arm_to_pose(pose_target)
                return True
            else:
                return False
        if step==1:
            pose_target = geometry_msgs.msg.Pose()
            print "Waiting for whycon"
            self.last_whyconpose=rospy.wait_for_message("/grab",geometry_msgs.msg.PoseStamped, timeout=90)
            print self.last_whyconpose
            pose_target = self.last_whyconpose.pose
            #pose_target.position.x = pose_target.position.x -0.03
            pose_target.position.z = pose_target.position.z +0.05
    #        pose_target.position.x = 0.769808727835
    #        pose_target.position.y = -0.259731727372
    #        pose_target.position.z = -0.103158764323
            pose_target.orientation.x = -0.154522394097
            pose_target.orientation.y = 0.983299780141
            pose_target.orientation.z = 0.0295583007957
            pose_target.orientation.w = 0.0914914146322
            rospy.sleep(0.5)
            pos_tar_st = geometry_msgs.msg.PoseStamped()
            pos_tar_st.header.stamp = rospy.Time.now()
            pos_tar_st.header.frame_id = 'base'
            pos_tar_st.pose = pose_target
            self.target_pose_pub.publish(pos_tar_st)
            rospy.sleep(0.5)
            print "Planning_to"
            print pose_target
            target_joints=self.__inverseKinematics('right',pose_target.position, pose_target.orientation)
            print target_joints
            if target_joints:
                self.move_arm_to_pose(pose_target)
                return True
            else:
                return False
        elif step==2:
            pose_target = geometry_msgs.msg.Pose()
            pose_target.position.x = 0.679869398247
            pose_target.position.y = -0.0196404042254
            pose_target.position.z = 0.12753082006
            pose_target.orientation.x = -0.49756487367
            pose_target.orientation.y = 0.858706247519
            pose_target.orientation.z = 0.043679577585
            pose_target.orientation.w = 0.11465108575


            rospy.sleep(0.5)
            pos_tar_st = geometry_msgs.msg.PoseStamped()
            pos_tar_st.header.stamp = rospy.Time.now()
            pos_tar_st.header.frame_id = 'base'
            pos_tar_st.pose = pose_target
            self.target_pose_pub.publish(pos_tar_st)
            rospy.sleep(0.5)
            print "Planning_to"
            print pose_target
            target_joints=self.__inverseKinematics('right',pose_target.position, pose_target.orientation)
            print target_joints
            if target_joints:
                self.move_arm_to_pose(pose_target)
                return True
            else:
                return False
        else:
            pose_target = geometry_msgs.msg.Pose()
            pose_target.position.x = 0.756359171414
            pose_target.position.y = -0.40
            pose_target.position.z = 0.24
            pose_target.orientation.x = -0.247796262522
            pose_target.orientation.y = 0.967815303441
            pose_target.orientation.z = 0.014473177124
            pose_target.orientation.w = 0.0414858752969
            rospy.sleep(0.5)
            pos_tar_st = geometry_msgs.msg.PoseStamped()
            pos_tar_st.header.stamp = rospy.Time.now()
            pos_tar_st.header.frame_id = 'base'
            pos_tar_st.pose = pose_target
            self.target_pose_pub.publish(pos_tar_st)
            rospy.sleep(0.5)
            print "Planning_to"
            print pose_target
            target_joints=self.__inverseKinematics('right',pose_target.position, pose_target.orientation)
            print target_joints
            if target_joints:
                self.move_arm_to_pose(pose_target)
                return True
            else:
                return False
        

    def __inverseKinematics(self, limb, point, orientation):
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        rospy.wait_for_service(ns)
        # time.sleep(2)
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose = geometry_msgs.msg.PoseStamped()
        pose.header=hdr
        pose.pose.position.x=point.x
        pose.pose.position.y=point.y
        pose.pose.position.z=point.z
        pose.pose.orientation.x=orientation.x
        pose.pose.orientation.y=orientation.y
        pose.pose.orientation.z=orientation.z
        pose.pose.orientation.w=orientation.w
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

    def move_arm_to_pose(self, pose):
        right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
        # pose = [0.28, -0.62, -0.32, 3.14/4, -3.14/2, 0]
        #pose = [0.815, -1.01, 0.321, 0.271, 0.653, -0.271, 0.653]
        #pose = [0.733690374074, -0.204190738435, -0.00903572458176, -0.410443965327, 0.909913908674, 0.0546822139767, 0.0245415077462]
#        pose_target = geometry_msgs.msg.Pose()
#        pose_target.orientation.x= -0.410443965327
#        pose_target.orientation.y= 0.909913908674
#        pose_target.orientation.z= 0.0546822139767
#        pose_target.orientation.w= 0.0245415077462
#        pose_target.position.x = 0.733690374074
#        pose_target.position.y = -0.204190738435
#        pose_target.position.z = -0.00903572458176
        #    pose_target = geometry_msgs.msg.Pose()
   
        print right_arm_group.get_current_pose()
    
        right_arm_group.set_pose_target(pose)
    
        right_arm_group.plan()
        right_arm_group.go(wait=True)
        

    def setup_moveit_commander(self):
        print "Initialising MoveIt!!!"
        self.moveit_robot = moveit_commander.RobotCommander()
        self.moveit_scene = moveit_commander.PlanningSceneInterface()
        self.moveit_right_group = moveit_commander.MoveGroupCommander("right_arm")
        #self.moveit_right_group.set_planner_id("RRTkConfigDefault")
        print "============ Reference frame: %s" % self.moveit_right_group.get_planning_frame()
        print "============ Effector: %s" % self.moveit_right_group.get_end_effector_link()
        print "============ Robot Groups:"
        print self.moveit_robot.get_group_names()
        print "============ Printing robot state"
        print self.moveit_robot.get_current_state()
        print "============"
        #self.moveit_scene.get_current_scene
    
#    def closeLeftGripper(self):
#        self.leftGripper.close()
#        
#    def openLeftGripper(self):
#        self.leftGripper.open()
#
#    def closeRightGripper(self):
#        self.rightGripper.close()
#        
#    def openRightGripper(self):
#        self.rightGripper.open()


if __name__ == '__main__':
    #moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("zoidbot_grab")
    zoidbot = BaxterGrab()
    rospy.spin()
    