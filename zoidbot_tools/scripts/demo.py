#!/usr/bin/python2

import sys
import os
from time import sleep

from threading import Timer
import rospy

import baxter_interface
#import baxter_interface


#import zoidbot_tools
from baxter_interface import CHECK_VERSION


from zoidbot_tools.puppet import *
from zoidbot_tools.player import *
from zoidbot_tools.recorder import *
from zoidbot_tools.io_interface import *



def reset_limbs():
    right_arm = baxter_interface.limb.Limb('right')
    left_arm = baxter_interface.limb.Limb('left')
    right_arm.move_to_neutral()
    left_arm.move_to_neutral()

    


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
        self.lmirror = Puppeteer('left')
        self.rmirror = Puppeteer('right')
        
        self.limbnames=['left','torso_left','right','torso_right']
        self.limbs=[]
        for i in self.limbnames:
            self.limbs.append(BaxIOinterface(i))
            
        self.rs.enable()
        for i in self.limbs:
            i.set_leds(True)
        t = Timer(1.0, self.readinginps)
        t.start()



    def execute_commands(self):
        if len(self.commands) >0:
            com = self.commands[0]
            self.commands.pop()
            if com['limb'] == 'torso_right':
                if com['button'] == 0:
                    self.new_mode = 'mirror_right'
                elif  com['button'] == 1:
                    self.new_mode = 'disable'
                elif com['button'] == 2:
                    self.new_mode = 'neutral'

    
            if com['limb'] == 'torso_left':
                if com['button'] == 0:
                    self.new_mode = 'mirror_left'
                elif  com['button'] == 1:
                    self.new_mode = 'disable'
                elif com['button'] == 2:
                    self.new_mode = 'neutral'
    
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
          
            if self.mode == 'disable':
                print "Enable"
                self.rs.enable()
            
            if self.mode == 'record':
                print "STOP RECORDING"
                self.recorder.stop()
                self.file_created = True

            if self.mode == 'mirror_left':
                print "STOP MIRROR LEFT"
                self.lmirror.set_stop()

            if self.mode == 'mirror_right':
                print "STOP MIRROR RIGHT"
                self.rmirror.set_stop()

            if self.new_mode == 'play': #and self.done_playing:
                if self.file_created:
                    print "Start Playing"
                    for i in self.limbs:
                        i.set_leds(False)
                        sleep(0.5)
                        i.set_blink(True, fast=True)
                    self.done_playing=False
                    player = JointPlayer('/tmp/baxter.traj')
                    if player.play_file():
                        self.done_playing=True
                        self.new_mode = 'iddle'
                else:
                    print "No trajectory recorded please record one and try again"
                    self.new_mode = 'iddle'
            #else:
                #print "Cancelling Trajectory"
            
            if self.new_mode == 'record':
                print "Start Recording"
                for i in self.limbs:
                    i.set_leds(False)
                    sleep(0.5)
                    i.set_blink(True)
                del self.recorder
                self.recorder = JointRecorder('/tmp/baxter.traj', 100)
                self.recorder.record()
    

            
            if self.new_mode == 'mirror_left':
                print "START MIRROR LEFT"
                for i in self.limbs:
                    i.set_leds(True)
                sleep(0.5)
                self.limbs[0].set_blink(True)
                self.limbs[1].set_blink(True)
                self.lmirror.puppet()

            if self.new_mode == 'mirror_right':
                for i in self.limbs:
                    i.set_leds(True)
                sleep(0.5)
                self.limbs[3].set_blink(True)
                self.limbs[2].set_blink(True)
                print "START MIRROR RIGHT"
                self.rmirror.puppet()


            if self.new_mode == 'neutral':            
                reset_limbs()
                print "limbs at neutral, setting idle mode"
                self.new_mode='iddle'


            if self.new_mode == 'iddle':
                print "IDDLE"
                for i in self.limbs:
                    i.set_blink(False)
                    sleep(0.5)
                    i.set_leds(True)
            
            if self.new_mode == 'disable':
                print "Disable"
                for i in self.limbs:
                    i.set_blink(False)
                    sleep(0.5)
                    i.set_leds(False)
                reset_limbs()
                self.rs.disable()

                
                
            self.mode=self.new_mode
            self.new_mode='none'

    def readinginps(self):
        command={}
        for i in self.limbs:
            if i.toggled:
                command['limb']= i.intname
                command['button']= i.b.index(1)
                self.commands.append(command)
            i.set_checked()
        

        if not self._killall_timers :
            self.execute_commands()
            t = Timer(1.0, self.readinginps)
            t.start()


    def _on_node_shutdown(self):
        self._killall_timers=True
        self.rs.disable()
        for i in self.limbs:
            i.set_leds(False)
        sleep(2)



if __name__ == '__main__':
    rospy.init_node('input_read')
    ps = BaxInputRead()
    rospy.spin()
#    sys.exit(main())
