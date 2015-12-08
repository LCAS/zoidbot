#!/usr/bin/env python

# Node for testing moveit stuff

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

class BaxterGrab:

    def __init__(self):
        
        right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
    
        # pose = [0.28, -0.62, -0.32, 3.14/4, -3.14/2, 0]
        #pose = [0.815, -1.01, 0.321, 0.271, 0.653, -0.271, 0.653]
        #pose = [0.733690374074, -0.204190738435, -0.00903572458176, -0.410443965327, 0.909913908674, 0.0546822139767, 0.0245415077462]
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.x= -0.410443965327
        pose_target.orientation.y= 0.909913908674
        pose_target.orientation.z= 0.0546822139767
        pose_target.orientation.w= 0.0245415077462
        pose_target.position.x = 0.733690374074
        pose_target.position.y = -0.204190738435
        pose_target.position.z = -0.00903572458176
        #    pose_target = geometry_msgs.msg.Pose()
    
   
        print right_arm_group.get_current_pose()
    
        right_arm_group.set_pose_target(pose)
    
        right_arm_group.plan()
        right_arm_group.go(wait=True)


if __name__=='__main__':
    rospy.loginfo("grabber")
    rospy.init_node('test_moveit', log_level=rospy.INFO)
    zoidbot = BaxterGrab()
