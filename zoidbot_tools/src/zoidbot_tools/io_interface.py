#!/usr/bin/python2

from time import sleep

from threading import Timer
import rospy

import baxter_interface
from baxter_interface import CHECK_VERSION


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
