#!/usr/bin/env python

"""
@author: claire

for spyder--># -*- coding: utf-8 -*-
for ROS----->#!/usr/bin/env python
"""

import numpy as np
import time

class QuadrotorParameters():
    """Stores information relevant to a quadcopter player; ***static***"""
    
    def __init__(self, m=0.5, k=3e-6, L=0.25, b=1e-7, I='Preset', g=9.81):
        """ Initializes State object"""
        self.m = m    # mass of quadcopter, kg
        self.k = k   # relates thrust to square of angular velocity (?)
        self.L = L   # quadcopter arm length, m
        self.b = b   # drag coefficient (?)
        self.g = g   # gravitational constant, m^2/s

        # inertia matrix
        if I == 'Preset':
            self.I = np.array([[5e-3, 0, 0], [0, 5e-3, 0], [0, 0, 5e-3]]) 
        else:
            self.I = I


class State():
    """tracks angular position and velocity of quadrotor"""

    def __init__(self):
        """ initalizes anglular state of quadrotor """
        self.thetas = np.zeros([3,1]) #angular orientation, ***make sure is same as startup/ gazebo standards
        self.thetadots = np.zeros([3,1]) #angular velocities [roll, pitch, yaw], radians


    def disturb(self):
        """ simulates a change in angular velocity (-5,5) to test that stablization or
        control code is working; possibly can deal with sensor errors and/or
        outside influences """

        # random deviation in angular velocity, degrees/sec
        deviation = 300
        self.thetadots = np.deg2rad(2 * deviation * np.random.rand(3,1) - deviation)
