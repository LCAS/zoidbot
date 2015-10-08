#!/usr/bin/python2

import sys
from time import sleep

from threading import Timer
import rospy

import baxter_interface
#import baxter_interface

#from baxter_examples import JointRecorder
from baxter_interface import CHECK_VERSION

def reset_limbs():
    right_arm = baxter_interface.limb.Limb('right')
    left_arm = baxter_interface.limb.Limb('left')
    right_arm.move_to_neutral()
    left_arm.move_to_neutral()

    

class Puppeteer(object):

    def __init__(self, limb, amplification=1.0):
        """
        Puppets one arm with the other.

        @param limb: the control arm used to puppet the other
        @param amplification: factor by which to amplify the arm movement
        """
        puppet_arm = {"left": "right", "right": "left"}
        self._control_limb = limb
        self._puppet_limb = puppet_arm[limb]
        self._control_arm = baxter_interface.limb.Limb(self._control_limb)
        self._puppet_arm = baxter_interface.limb.Limb(self._puppet_limb)
        self._amp = amplification

        self.stop=True
#        print("Getting robot state... ")
#        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
#        self._init_state = self._rs.state().enabled
#        print("Enabling robot... ")
#        self._rs.enable()

    def _reset_control_modes(self):
        rate = rospy.Rate(100)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._control_arm.exit_control_mode()
            self._puppet_arm.exit_control_mode()
            rate.sleep()
        return True

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose.
        """
        print("Moving to neutral pose...")
        self._control_arm.move_to_neutral()
        self._puppet_arm.move_to_neutral()

    def set_stop(self):
        print("\nExiting example...")
        #return to normal
        self._reset_control_modes()
        self.set_neutral()
        self.stop=True
        #return True

    def puppet(self):
        """

        """
        self.set_neutral()
        self.rate = 1.0/100.0

        self.control_joint_names = self._control_arm.joint_names()
        self.puppet_joint_names = self._puppet_arm.joint_names()

        print ("Puppeting:\n"
              "  Grab %s cuff and move arm.\n"
              "  Press Ctrl-C to stop...") % (self._control_limb,)
        
        self.stop=False
        t = Timer(self.rate, self.move)
        t.start()
    
    def move(self):
        if not self.stop:
            cmd = {}
            for idx, name in enumerate(self.puppet_joint_names):
                v = self._control_arm.joint_velocity(
                    self.control_joint_names[idx])
                if name[-2:] in ('s0', 'e0', 'w0', 'w2'):
                    v = -v
                cmd[name] = v * self._amp
            self._puppet_arm.set_joint_velocities(cmd)
            t = Timer(self.rate, self.move)
            t.start()
        #rate.sleep()



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




class BaxIOinterface(object):

    def __init__(self, intname) :
        self.intname =intname
        self.toggled = False
        self.blinking = False
        self.nav = baxter_interface.Navigator(intname)
        self.nav.button0_changed.connect(self.b0_pressed)
        self.nav.button1_changed.connect(self.b1_pressed)
        self.nav.button2_changed.connect(self.b2_pressed)
        self.b=[0,0,0]

    def set_leds(self, v):
        self.blinking=False
        self.nav.inner_led = v
        self.nav.outer_led = v

    def toggle_leds(self):
        self.nav.inner_led = not self.nav.inner_led 
        self.nav.outer_led = not self.nav.outer_led
        if self.blinking:
            t = Timer(self.period, self.toggle_leds)
            t.start()        
        
    def set_blink(self, v, fast=False):
        if fast:
            self.period = 0.1
        else:
            self.period = 0.5
        self.blinking = v
        if v:
            t = Timer(self.period, self.toggle_leds)
            t.start()


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
        
        self.recorder = JointRecorder('/tmp/baxter.traj', 20)
        self.player = JointPlayer('/tmp/baxter.traj')
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
                self.recorder.stop    
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
                    if self.player.play_file():
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
                self.recorder = JointRecorder('/tmp/baxter.traj', 20)
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
        for i in self.limbs:
            i.set_leds(False)
        sleep(2)



if __name__ == '__main__':
    rospy.init_node('input_read')
    ps = BaxInputRead()
    rospy.spin()
#    sys.exit(main())
