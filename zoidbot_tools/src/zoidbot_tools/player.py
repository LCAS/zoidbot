#!/usr/bin/python2

import sys
import os
from time import sleep

from threading import Timer
import rospy

import baxter_interface
from baxter_interface import CHECK_VERSION

class JointPlayer(object):
    def __init__(self, filename, loops=1):
        self.filename=filename
        self.loops=loops
    
    
    def try_float(self, x):
        try:
            return float(x)
        except ValueError:
            return None


    def clean_line(self, line, names):
        """
        Cleans a single line of recorded joint positions
    
        @param line: the line described in a list to process
        @param names: joint name keys
        """
        #convert the line of strings to a float or None
        line = [self.try_float(x) for x in line.rstrip().split(',')]
        #zip the values with the joint names
        combined = zip(names[1:], line[1:])
        #take out any tuples that have a none value
        cleaned = [x for x in combined if x[1] is not None]
        #convert it to a dictionary with only valid commands
        command = dict(cleaned)
        left_command = dict((key, command[key]) for key in command.keys()
                            if key[:-2] == 'left_')
        right_command = dict((key, command[key]) for key in command.keys()
                             if key[:-2] == 'right_')
        return (command, left_command, right_command, line)


    def play_file(self):
        """
        Loops through csv file
    
        @param filename: the file to play
        @param loops: number of times to loop
                      values < 0 mean 'infinite'
    
        Does not loop indefinitely, but only until the file is read
        and processed. Reads each line, split up in columns and
        formats each line into a controller command in the form of
        name/value pairs. Names come from the column headers
        first column is the time stamp
        """
        filename = self.filename
        loops = self.loops
        
        
        left = baxter_interface.Limb('left')
        right = baxter_interface.Limb('right')
        grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
        grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
        rate = rospy.Rate(1000)
    
        if grip_left.error():
            grip_left.reset()
        if grip_right.error():
            grip_right.reset()
        if (not grip_left.calibrated() and
            grip_left.type() != 'custom'):
            grip_left.calibrate()
        if (not grip_right.calibrated() and
            grip_right.type() != 'custom'):
            grip_right.calibrate()
    
        print("Playing back: %s" % (filename,))
        with open(filename, 'r') as f:
            lines = f.readlines()
        keys = lines[0].rstrip().split(',')
    
        l = 0
#        # If specified, repeat the file playback 'loops' number of times
#        while loops < 1 or l < loops:
#            i = 0
        l += 1
        print("Moving to start position...")
        i = 0
        _cmd, lcmd_start, rcmd_start, _raw = self.clean_line(lines[1], keys)
        left.move_to_joint_positions(lcmd_start)
        right.move_to_joint_positions(rcmd_start)
        start_time = rospy.get_time()
        for values in lines[1:]:
            i += 1
            loopstr = str(loops) if loops > 0 else "forever"
            sys.stdout.write("\r Record %d of %d, loop %d of %s" %
                             (i, len(lines) - 1, l, loopstr))
            sys.stdout.flush()

            cmd, lcmd, rcmd, values = self.clean_line(values, keys)
            #command this set of commands until the next frame
            while (rospy.get_time() - start_time) < values[0]:
                if rospy.is_shutdown():
                    print("\n Aborting - ROS shutdown")
                    return False
                if len(lcmd):
                    left.set_joint_positions(lcmd)
                if len(rcmd):
                    right.set_joint_positions(rcmd)
                if ('left_gripper' in cmd and
                    grip_left.type() != 'custom'):
                    grip_left.command_position(cmd['left_gripper'])
                if ('right_gripper' in cmd and
                    grip_right.type() != 'custom'):
                    grip_right.command_position(cmd['right_gripper'])
                rate.sleep()
            print 
        #print "DONEEEE"
        return True

