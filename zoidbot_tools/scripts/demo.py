#!/usr/bin/python2

import sys
from time import sleep

from threading import Timer
import rospy

import baxter_interface
#import baxter_interface

#from baxter_examples import JointRecorder
from baxter_interface import CHECK_VERSION


class BaxIOinterface(object):

    def __init__(self, intname) :
        self.intname =intname
        self.toggled = False
        self.nav = baxter_interface.Navigator(intname)
        self.nav.button0_changed.connect(self.b0_pressed)
        self.nav.button1_changed.connect(self.b1_pressed)
        self.nav.button2_changed.connect(self.b2_pressed)
        self.b=[0,0,0]

    def b0_pressed(self, v):
        self.toggled = True
        self.b[0]=1

    def b1_pressed(self, v):
        self.toggled = True
        self.b[1]=1

    def b2_pressed(self, v):
        self.toggled = True
        self.b[2]=1
    
    def set_checked(self):
        self.b=[0,0,0]
        self.toggled = False


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
            self.file.close
        #self._rate.sleep()




class BaxInputRead(object):
    _killall_timers=False

    def __init__(self) :
        self.commands=[]
        self.mode = 'iddle'
        self.new_mode = 'none'
        self.file_created=False
        self.done_playing=False
        rospy.on_shutdown(self._on_node_shutdown)
        self.rs = baxter_interface.RobotEnable(CHECK_VERSION)
        
        self.recorder = JointRecorder('/tmp/baxter.traj', 100)
        
        self.limbnames=['left','torso_left','right','torso_right']
        self.limbs=[]
        for i in self.limbnames:
            self.limbs.append(BaxIOinterface(i))

        self.rs.enable()
        
        t = Timer(1.0, self.readinginps)
        t.start()



    def execute_commands(self):
        if len(self.commands) >0:
            com = self.commands[0]
            self.commands.pop()
            if com['limb'] == 'torso_right':
                if com['button'] == 0:
                    self.new_mode = 'mirror_right'
                else:
                    self.new_mode = 'iddle'
    
            if com['limb'] == 'torso_left':
                if com['button'] == 0:
                    self.new_mode = 'mirror_left'
                else:
                    self.new_mode = 'iddle'
    
            if com['limb'] == 'left':
                if com['button'] == 0:
                    self.new_mode = 'iddle'
                elif com['button'] == 2:
                    self.new_mode = 'record'
                elif com['button'] == 1:
                    self.new_mode = 'play'
    
            if com['limb'] == 'right':
                if com['button'] == 0:
                    self.new_mode = 'iddle'
                elif com['button'] == 2:
                    self.new_mode = 'record'
                elif com['button'] == 1:
                    self.new_mode = 'play'    
    
        self.set_mode()


    def set_mode(self):
        if self.new_mode != self.mode and self.new_mode != 'none' :
            print "changing from %s to %s" %(self.mode, self.new_mode)
            if self.mode == 'record':
                print "STOP RECORDING"
                self.recorder.stop    
                self.file_created = True

            if self.mode == 'mirror_left':
                print "STOP MIRROR LEFT"

            if self.mode == 'mirror_right':
                print "STOP MIRROR RIGHT"

            if self.new_mode == 'play': #and self.done_playing:
                if self.file_created:
                    print "Start Playing"
                else:
                    print "No trajectory recorded please record one and try again"
                    self.new_mode = 'iddle'
            #else:
                #print "Cancelling Trajectory"
            
            if self.new_mode == 'record':
                print "Start Recording"
                del self.recorder
                self.recorder = JointRecorder('/tmp/baxter.traj', 100)
                self.recorder.record()
    

            
            if self.new_mode == 'mirror_left':
                print "START MIRROR LEFT"

            if self.new_mode == 'mirror_right':
                print "START MIRROR RIGHT"            

            if self.new_mode == 'iddle':
                print "IDDLE"
            
            self.mode=self.new_mode
            self.new_mode='none'

    def readinginps(self):
        command={}
        for i in self.limbs:
            if i.toggled:
                #print "Button %d Pressed in %s" %(i.b.index(1),i.intname)
                command['limb']= i.intname
                command['button']= i.b.index(1)
                #print command
                self.commands.append(command)
            i.set_checked()
        

        if not self._killall_timers :
            self.execute_commands()
            t = Timer(1.0, self.readinginps)
            t.start()


    def _on_node_shutdown(self):
        self._killall_timers=True
        self.rs.disable()
        sleep(2)



if __name__ == '__main__':
    rospy.init_node('input_read')
    ps = BaxInputRead()
    rospy.spin()
#    sys.exit(main())
