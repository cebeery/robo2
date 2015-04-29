#!/usr/bin/env python

"""
@author: claire

for spyder--># -*- coding: utf-8 -*-
for ROS----->#!/usr/bin/env python
"""

import numpy as np
import time
import trajectory_utils as tu
import bisect

''' FILE STRUCTURE '''
# -> class QuadcopterParameters
#   -> def update
# -> class State
#   -> def disturb
#   -> def update
# -> class Model
#   -> is a State
#   -> has a QuadrotorParameters

class QuadrotorParameters():
    """Stores information relevant to a quadcopter player's static physics"""
    
    def __init__(self):
        """ 
        Initializes State object
        'm', mass, kg
        'k', relates thrust to square of angular velocity????, ????
        'L', arm length, m
        'b', drag coefficient, ??
        'g', acceleration due to gravity, m^2/s 
        'I', inertia matrix, 3x3 array of ??
        """
        self.m = 1.477    
        self.k = 3e-6 #not changed   
        self.L = 0.25 #not changed  
        self.b = 1e-7 #not changed   
        self.g = 9.81  
        self.I = np.array([[1.152e-2, 0.0, 0.0], [0.0, 1.152e-1, 0.0], [0.0, 0.0, 2.18e-2]]) 

    def update(self, file_name):
        """ 
        Loads parameter from a saved file
        INPUTS:  file_name, name of file with learned parameters, string
        """
        pass #fix


class State():
    """Tracks orientation, position, and velocity of quadrotor"""

    def __init__(self):
        """
        Initalizes anglular state of quadrotor 
        'locations', position in global x,y,z-coordinate system, 3x1 array of ??
        'thetas', angular orientations, 3x1 array of radians?
        'thetadots', angular velocities, 3x1 array of radians/sec?
        """
        self.locations = np.zeros([3,1])  
        self.thetas = np.zeros([3,1]) 
        self.prev_thetas = np.zeros([3,1])  
        self.thetadots = np.zeros([3,1]) 


    def disturb(self):
        """ simulates a change in angular velocity (-5,5) to test that stablization or
        control code is working; possibly can deal with sensor errors and/or
        outside influences """

        # random deviation in angular velocity, degrees/sec
        deviation = 300
        self.thetadots = np.deg2rad(2 * deviation * np.random.rand(3,1) - deviation)

    def update(self, thetas):
        """updates quadcopter pose and location"""
        self.prev_thetas = self.thetas
        self.thetas = thetas
        self.thetadots = (self.thetas - self.prev_thetas) * 0.1
        
        


class Model(State):
    """Tracks the static and loction/pose attributes of the simulated quadrotor """
    
    def __init__(self, traj_file=None):
        """ 
        Initializes State object
        state, stores pose and location of quadrotor, see State
        'trajectory', evenly-spaced locations on desired path, ???
        'para', quadrotor learned physical parameters, see QuadcopterParameters
        """
        State.__init__(self)   
        self.trajectory = tu.makeTrajectory(traj_file)   
        self.params = QuadrotorParameters() 

    def key_vel(self, elasped_time):
        """ """
        times = self.trajectory.keyframes['t']
        d_thetadots = self.trajectory.keyframes['thdot']

        index = bisect.bisect_left(times, elasped_time) #??
        setpoint = np.zeros([3,1])

        if (index >= len(times)-1): # done with maneuver
            running = False
        else: 
            setpoint[2,0] = d_thetadots[index]/36
            running = True

        return [setpoint, running]

