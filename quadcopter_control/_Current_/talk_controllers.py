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
from gazebo_msgs.msg import ModelStates

''' FILE STRUCTURE '''
# -> class TranslationalController
#   -> has QuadrotorParameters
#   -> def torques
#   -> def angular_acceleration
#   -> def findW
#   -> def theta2omega
#   -> def omega2thetadot
#   -> def translate_model
# -> class ROSTalker
#   -> def callback
#   -> def rise
#   -> def talk
#   -> def listen

class TranslationalController():

    def __init__(self):
         """ make main model location, calls state and controller""" 
         self.para = QuadrotorParameters()

    def torques(self, rotors):
        """Compute torques, given current inputs, length, drag coefficient, and thrust coefficient """
                
        tau = np.array([
        [self.para.L * self.para.k * (rotors[0,0] - rotors[2,0])],
        [self.para.L * self.para.k * (rotors[1,0] - rotors[3,0])],
        [self.para.b * (rotors[0,0] - rotors[1,0] + rotors[2,0] - rotors[3,0])]
        ])

        return tau
        

    def angular_acceleration(self, rotors, omega):
        """ Compute angular acceleration in body frame """
        tau = self.torques(rotors)
        inv_I = np.linalg.inv(self.para.I)
        
        cross = np.cross(omega.flatten(), np.dot(self.para.I, omega).flatten())
        omegadot =  np.dot(inv_I, tau - np.reshape(cross,[3,1]))
        return omegadot


    def findW(self, thetas):
        """ ???? what is w ???
        ***math came from?*** """
      
        phi =   thetas[0,0]
        theta = thetas[1,0]
        psi =   thetas[2,0] #***not used ***??
    
        W = np.array([
            [1,      0,                 -1 * np.sin(theta)],
            [0,      np.cos(phi),       np.cos(theta)*np.sin(phi)],
            [0,      -1*np.sin(phi),    np.cos(theta)*np.cos(phi)]
            ])
        
        return W


    def thetadot2omega(self, thetadots, thetas):
        """ convert derivatives of roll, pitch, yaw to omega
        ***what is omega***
        ***math came from?*** """
    
        W = self.findW(thetas)
        omega = np.dot(W,thetadots)
        
        return omega
        
        
    def omega2thetadot(self, omega, thetas):
        """ convert omega to derivatives of roll, pitch, yaw
        ***what is omega***
        ***math came from?*** """
    
        W = self.findW(thetas)
        inv_W = np.linalg.inv(W)
        thetadots = np.dot(inv_W, omega)

        return thetadots


    def translate_model(self, rotors, thetadots, thetas, dt):
         """ translates desired rotor speeds into new angular velocities"""
         omega = self.thetadot2omega(thetadots, thetas)
         omegadot = self.angular_acceleration(rotors, omega)

         # Advance system state.
         omega = omega + dt * omegadot        
         thetadots = self.omega2thetadot(omega, thetas)

         return thetadots


class ROSTalker():
    """ """ 
    
    def __init__(self):
        """ """
        # initialize node and publisher
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)        
        rospy.init_node('talker', anonymous=True)

        #attributes
        self.subscriber_data = None
        self.rate = rospy.Rate(10) #10Hz
        self.twist = Twist() #sets velocity commands to 0
        
        # start subscriber
        rospy.Subscriber('gazebo/model_states', ModelStates, self.callback)
        
        # initialize twist commands, note: inital statement is unpublished 
        self.pub.publish(self.twist)
	self.rate.sleep()
        

    def callback(self, msg):
	""" """
        thetas = np.zeros([3,1])
        quad = msg.pose[1]

        thetas[0,0] = quad.orientation.x
        thetas[1,0] = quad.orientation.y
        thetas[2,0] = quad.orientation.z

        self.subscriber_data = thetas
        

    def rise(self): 
     	
        #initalize rise
        start_time = time.time()
    
      	# start ascending
        rospy.loginfo('Start ascending')
        self.twist.linear.z = 1
        self.pub.publish(self.twist)
        self.rate.sleep() 
        time.sleep(3) #rise time
        
        # stop rising
        rospy.loginfo('Start hovering')
        self.twist.linear.z = 0
        self.pub.publish(self.twist)
	self.rate.sleep() 
       
    def talk(self, ang_vel):
        self.twist = Twist()
        self.twist.linear.x = 1
        self.twist.angular.z = ang_vel[2,0]
        self.pub.publish(self.twist)  
	self.rate.sleep() 

    def stop(self):
        self.twist = Twist()
        self.pub.publish(self.twist)  
	self.rate.sleep() 