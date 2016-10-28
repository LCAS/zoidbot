#!/usr/bin/env python

# Node for testing moveit stuff

import sys
import rospy
#import moveit_commander
#import moveit_msgs.msg
import zoidbot_tools.srv


def cleanning_server():
    rospy.wait_for_service('/pick_block_right')
    try:
        clean = rospy.ServiceProxy('/pick_block_right', zoidbot_tools.srv.PickBlock)
        resp = clean()
        return resp.success
    except rospy.ServiceException :
        return False


if __name__=='__main__':
    rospy.loginfo("Starting cleaning")
    rospy.init_node('cleaning_table', log_level=rospy.INFO)
    retry=0
    resp=True
    while resp and not rospy.is_shutdown():
        resp=cleanning_server()
        print resp
        if not resp and retry<5:
            retry+=1
            resp=True