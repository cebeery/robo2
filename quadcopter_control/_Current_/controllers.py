#!/usr/bin/env python

"""
@author: claire

for spyder--># -*- coding: utf-8 -*-
for ROS----->#!/usr/bin/env python
"""
from model import QuadrotorParameters, State
import numpy as np
import time
import rospy
from geometry_msgs.msg import Twist

''' FILE STRUCTURE '''
# -> class PIDController
#   -> def update
# -> class RotorDynamicsModel
#   -> def rotor_speeds

class PidController:
    """ Creates a PID Controller that references the model/state information"""
    
    def __init__(self, Kd=4, Kp=3, Ki=5.5):
        """ Initalizes PID Controller object with PID constants """
        
        self.dt = 0.1 # timestep, ***static***
        self.proportional = np.zeros([3,1])
        self.integral = np.zeros([3,1])
        
        self.Kd = Kd
        self.Kp = Kp
        self.Ki = Ki


    def update(self, model, setpoints=np.zeros([3,1])):
        """ Performs PID update against setpoint, adjusts thetadot of 
        rotors, updates intregral states (no sensors)??"""
    
        # unpack quadrotor parameters
        g = model.params.g
        m = model.params.m
        k = model.params.k

        # unpack current quadcopter state
        thetadots = model.thetadots
        
        #prevent intergral windup
        if max(self.integral) > 0.01:
            self.integral = np.zeros([3,1])

        # Compute total thrust. ***
        total = m * g / k / np.cos(self.proportional[0,0]) * np.cos(self.proportional[2,0])

        # Compute error against zero and adjust with PID 
        err = self.Kd * (thetadots-setpoints) + self.Kp * self.proportional - self.Ki * self.integral

        # Update PID states ****does not ref real model directly*****.
        self.proportional = self.proportional + self.dt * (thetadots-setpoints)
        self.integral = self.integral + self.dt * self.proportional

        return [err, total]

class RotorDynamicsModel():

    def __init__(self, model):
         """ make main model location, calls state and controller""" 
         self.model = model

    def rotor_speeds(self, err, total, model):
        """ compute quadrotor rotor velocities with controller correction """
        
        # unpack error
        e1 = err[0,0]
        e2 = err[1,0]
        e3 = err[2,0]
 
        # unpack quadrotor parameters
        Ix = model.params.I[0,0]
        Iy = model.params.I[1,1]
        Iz = model.params.I[2,2]
        k = model.params.k
        L = model.params.L
        b = model.params.b

        # compute quadcopter rotor speed inputs
        rotor_speed = np.zeros([4,1])
        rotor_speed[0,0] = total/4 -(2 * b * e1 * Ix + e3 * Iz * k * L)/(4 * b * k * L)
        rotor_speed[1,0] = total/4 + e3 * Iz/(4 * b) - (e2 * Iy)/(2 * k * L)
        rotor_speed[2,0] = total/4 -(-2 * b * e1 * Ix + e3 * Iz * k * L)/(4 * b * k * L)
        rotor_speed[3,0] = total/4 + e3 * Iz/(4 * b) + (e2 * Iy)/(2 * k * L)

        return rotor_speed

