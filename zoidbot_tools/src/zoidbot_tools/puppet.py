#!/usr/bin/python2

from time import sleep

from threading import Timer
import rospy

import baxter_interface
from baxter_interface import CHECK_VERSION

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
        self._gripper_control = baxter_interface.Gripper(self._control_limb, CHECK_VERSION)
        self._gripper_puppet = baxter_interface.Gripper(self._puppet_limb, CHECK_VERSION)
        if self._control_limb == 'left':
            self._io_control_lower = baxter_interface.DigitalIO('left_lower_button')
            self._io_control_upper = baxter_interface.DigitalIO('left_upper_button')
        else:
            self._io_control_lower = baxter_interface.DigitalIO('right_lower_button')
            self._io_control_upper = baxter_interface.DigitalIO('right_upper_button')
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
            if self._io_control_lower.state:
                self._gripper_puppet.open()
                self._gripper_control.open()
            elif self._io_control_upper.state:
                self._gripper_puppet.close()
                self._gripper_control.close()            
            
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
