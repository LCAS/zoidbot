#!/usr/bin/python2

import sys
import os
from time import sleep

from threading import Timer
import rospy

import baxter_interface
from baxter_interface import CHECK_VERSION


class JointRecorder(object):
    def __init__(self, filename, rate):
        """
        Records joint data to a file at a specified rate.
        """
        self._filename = filename
        self._raw_rate = rate
        self._rate = rospy.Rate(rate)
        self._start_time = rospy.get_time()
        self._done = False

        self._limb_left = baxter_interface.Limb("left")
        self._limb_right = baxter_interface.Limb("right")
        self._gripper_left = baxter_interface.Gripper("left", CHECK_VERSION)
        self._gripper_right = baxter_interface.Gripper("right", CHECK_VERSION)
        self._io_left_lower = baxter_interface.DigitalIO('left_lower_button')
        self._io_left_upper = baxter_interface.DigitalIO('left_upper_button')
        self._io_right_lower = baxter_interface.DigitalIO('right_lower_button')
        self._io_right_upper = baxter_interface.DigitalIO('right_upper_button')

        # Verify Grippers Have No Errors and are Calibrated
        if self._gripper_left.error():
            self._gripper_left.reset()
        if self._gripper_right.error():
            self._gripper_right.reset()
        if (not self._gripper_left.calibrated() and
            self._gripper_left.type() != 'custom'):
            self._gripper_left.calibrate()
        if (not self._gripper_right.calibrated() and
            self._gripper_right.type() != 'custom'):
            self._gripper_right.calibrate()

    def _time_stamp(self):
        return rospy.get_time() - self._start_time

    def stop(self):
        """
        Stop recording.
        """
        print "!!!! Closing Record File !!!!"
        #self.file.close()
        self._done = True

    def done(self):
        """
        Return whether or not recording is done.
        """
        if rospy.is_shutdown():
            self.stop()
        return self._done

    def record(self):
        """
        Records the current joint positions to a csv file if outputFilename was
        provided at construction this function will record the latest set of
        joint angles in a csv format.

        This function does not test to see if a file exists and will overwrite
        existing files.
        """
        if self._filename:
            
            if os.path.exists(self._filename):
                os.remove(self._filename)
            self.joints_left = self._limb_left.joint_names()
            self.joints_right = self._limb_right.joint_names()
            
            self.file = open(self._filename, 'w')
            self.file.write('time,')
            self.file.write(','.join([j for j in self.joints_left]) + ',')
            self.file.write('left_gripper,')
            self.file.write(','.join([j for j in self.joints_right]) + ',')
            self.file.write('right_gripper\n')

            t = Timer(1.0/self._raw_rate, self.save)
            t.start()

    def save(self):
        # Look for gripper button presses
        if self._io_left_lower.state:
            self._gripper_left.open()
        elif self._io_left_upper.state:
            self._gripper_left.close()
        if self._io_right_lower.state:
            self._gripper_right.open()
        elif self._io_right_upper.state:
            self._gripper_right.close()
        angles_left = [self._limb_left.joint_angle(j)
                       for j in self.joints_left]
        angles_right = [self._limb_right.joint_angle(j)
                        for j in self.joints_right]

        self.file.write("%f," % (self._time_stamp(),))

        self.file.write(','.join([str(x) for x in angles_left]) + ',')
        self.file.write(str(self._gripper_left.position()) + ',')

        self.file.write(','.join([str(x) for x in angles_right]) + ',')
        self.file.write(str(self._gripper_right.position()) + '\n')

        if not self.done():
            t = Timer(1.0/self._raw_rate, self.save)
            t.start()
        else:
            print "!!!! Closing Record File !!!!"
            self.file.close()
        #self._rate.sleep()

