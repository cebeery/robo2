#!/usr/bin/env python

"""
Created on Wed Apr  1 08:07:56 2015

@author: claire
Extended variation of 
https://github.com/gibiansky/experiments/blob/master/quadcopter/matlab/ 

for spyder--># -*- coding: utf-8 -*-
for ROS----->#!/usr/bin/env python
"""

import rospy
from geometry_msgs.msg import Twist
import numpy as np
import time


class QuadrotorParameters():
    """Stores information relevant to a quadcopter player; ***static***"""
    
    def __init__(self):
        """ Initializes State object"""
        self.m = 0.5    # mass of quadcopter, kg
        self.g = 9.81   # gravitational constant, m^2/s
        self.k = 3e-6   # relates thrust to square of angular velocity (?)
        self.L = 0.25   # quadcopter arm length, m
        self.b = 1e-7   # drag coefficient (?)
        self.I = np.array([[5e-3, 0, 0], [0, 5e-3, 0], [0, 0, 5e-3]]) # inertia matrix


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


class PidController:
    """ Creates a PID Controller that references the player information"""
    
    def __init__(self, Kd=4, Kp=3, Ki=5.5):
        """ Initalizes PID Controller object with PID constants """
        
        self.dt = 0.1 # timestep, ***static***
        self.proportional = np.zeros([3,1])
        self.integral = np.zeros([3,1])
        
        self.Kd = Kd
        self.Kp = Kp
        self.Ki = Ki


    def update(self, model):
        """ Performs PID update against setpoint [0,0,0], adjusts thetadot of 
        rotors, updates intregral states (no sensors)??"""
    
        # unpack quadrotor parameters
        g = model.quad.g
        m = model.quad.m
        k = model.quad.k
        
        #prevent intergral windup
        if max(self.integral) > 0.01:
            self.integral = np.zeros([3,1])

        # Compute total thrust. ***
        total = m * g / k / np.cos(self.proportional[0,0]) * np.cos(self.proportional[2,0])

        # Compute error against zero and adjust with PID 
        err = self.Kd * model.state.thetadots + self.Kp * self.proportional - self.Ki * self.integral

        # Update controller state.
        self.proportional = self.proportional + self.dt * model.state.thetadots
        self.integral = self.integral + self.dt * self.proportional

        return [err, total]
        
        
class Model():

    def __init__(self, state, quad):
         """ make main model location, calls state and controller""" 
         self.state = state
         self.quad = quad


    def torques(self, rotors):
        """Compute torques, given current inputs, length, drag coefficient, and thrust coefficient """
                
        tau = np.array([
        [self.quad.L * self.quad.k * (rotors[0,0] - rotors[2,0])],
        [self.quad.L * self.quad.k * (rotors[1,0] - rotors[3,0])],
        [self.quad.b * (rotors[0,0] - rotors[1,0] + rotors[2,0] - rotors[3,0])]
        ])

        return tau
        

    def angular_acceleration(self, rotors, omega):
        """ Compute angular acceleration in body frame """
        tau = self.torques(rotors)
        inv_I = np.linalg.inv(self.quad.I)
        
        cross = np.cross(omega.flatten(), np.dot(self.quad.I, omega).flatten())
        omegadot =  np.dot(inv_I, tau - np.reshape(cross,[3,1]))
        return omegadot


    def findW(self):
        """ ???? what is w ???
        ***math came from?*** """
      
        phi =   self.state.thetas[0,0]
        theta = self.state.thetas[1,0]
        psi =   self.state.thetas[2,0] #***not used ***??
    
        W = np.array([
            [1,      0,                 -1 * np.sin(theta)],
            [0,      np.cos(phi),       np.cos(theta)*np.sin(phi)],
            [0,      -1*np.sin(phi),    np.cos(theta)*np.cos(phi)]
            ])
        
        return W


    def thetadot2omega(self):
        """ convert derivatives of roll, pitch, yaw to omega
        ***what is omega***
        ***math came from?*** """
    
        W = self.findW()
        omega = np.dot(W,self.state.thetadots)
        
        return omega
        
        
    def omega2thetadot(self, omega):
        """ convert omega to derivatives of roll, pitch, yaw
        ***what is omega***
        ***math came from?*** """
    
        W = self.findW()
        inv_W = np.linalg.inv(W)
        self.state.thetadots = np.dot(inv_W, omega)

#        % Arbitrary test input.
#        function in = input(t)
#            in = zeros(4, 1);
#            in(:) = 700;
#            in(1) = in(1) + 150;
#            in(3) = in(3) + 150;
#            in = in .^ 2;
#        end

    def translate_model(self, rotors, dt):
         """ translates desired rotor speeds into new angular velocities"""
         omega = self.thetadot2omega()
         omegadot = self.angular_acceleration(rotors, omega)

         # Advance system state.
         omega = omega + dt * omegadot        
         self.omega2thetadot(omega)


    def rotor_speeds(self, err, total):
        """ compute quadrotor rotor velocities with controller correction """
        
        # unpack error
        e1 = err[0,0]
        e2 = err[1,0]
        e3 = err[2,0]
 
        # unpack quadrotor parameters
        Ix = self.quad.I[0,0]
        Iy = self.quad.I[1,1]
        Iz = self.quad.I[2,2]
        k = self.quad.k
        L = self.quad.L
        b = self.quad.b

        # compute quadcopter rotor speed inputs
        rotor_speed = np.zeros([4,1])
        rotor_speed[0,0] = total/4 -(2 * b * e1 * Ix + e3 * Iz * k * L)/(4 * b * k * L)
        rotor_speed[1,0] = total/4 + e3 * Iz/(4 * b) - (e2 * Iy)/(2 * k * L)
        rotor_speed[2,0] = total/4 -(-2 * b * e1 * Ix + e3 * Iz * k * L)/(4 * b * k * L)
        rotor_speed[3,0] = total/4 + e3 * Iz/(4 * b) + (e2 * Iy)/(2 * k * L)

        return rotor_speed


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
#        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        
        # initialize twist commands, note: inital statement is unpublished 
        self.pub.publish(self.twist)
        

    def callback(self, msg):
	""" """
        self.subscriber_data = [[msg.linear.x,msg.linear.y,msg.linear.z],
		                [msg.angular.x,msg.angular.y,msg.angular.z]]

        rospy.loginfo(self.subscriber_data[0])
        rospy.loginfo(self.subscriber_data[1])
        
    def rise(self,start_time): 
     	    
      	# start ascending
	self.rate.sleep()
        self.twist.linear.z = 1
        self.pub.publish(self.twist) 
        time.sleep(5) #rise time

        # stop rising
        self.rate.sleep()
        rospy.loginfo('Start hovering')
        self.twist.linear.z = 0
        self.pub.publish(self.twist)
	self.rate.sleep() #needed suffix 

	#rospy.spin() will not allow code continuation
	#while not rospy.is_shutdown() inelegant for short sections
	#    requires additional parameter to track
	#self.rate.sleep() redundant with current command orders
	#    restructure? (rate.sleep() at end of each instead of beginning)

        
    def talk(self, ang_vel):
        self.rate.sleep() 
        self.twist = Twist()
#        self.twist.angular.x = ang_vel[0,0] 
#        self.twist.angular.y = ang_vel[1,0]
        self.twist.angular.z = ang_vel[2,0]
        self.pub.publish(self.twist)  
	self.rate.sleep() #restruct reminder**
        

if __name__ == "__main__":
    
    # create ROS node and functions
    rt = ROSTalker() 
   
    # raise quadcopter
    rt.rise(time.time())  
    
    #create model and state
    state = State()
    quad = QuadrotrParameters()
    model = Model(state,quad)
    model.state.disturb() #disturb system for testing
    
    #create PID controller
    p = PidController()
    
    rospy.loginfo('Start Reached')
    
    
#    #----test----#
#    
#    #create empties for tracking
#    errors  = np.zeros([20,3])
#    ang_vel = np.zeros([20,3])
#   
#    for x in range(0, 20):
#        # find error 
#        [err, total] = p.update(model)
#        errors[x,:] = err.flatten()
#       
#        # run model for rotor speeds
#        raw_data = model.rotor_speeds(err, total)
#
#        # translate model 
#        model.translate_model(raw_data, p.dt)
#        mod_data = model.state.thetadots
#        ang_vel[x,:] = mod_data.flatten()
#    
#    print('errors')
#    print(errors)
#    print('ang_vel')
#    print(ang_vel)
#    
#    #----test----#
    
    #behaviors run loop
    while not rospy.is_shutdown():
       
     	# *** collect current data ***
     	if rt.subscriber_data is not None:
     	   #rospy.loginfo(rt.subscriber_data)
     	   rt.subscriber_data = None
       
     	# *** translate data *** ***does not look at real thetadot
     	#currently tracking thetadot through model assumption, not topics
     	#theta position never updated from zero
     	#*** does thetadot need to be tracked or is it pullable from a gazebo topic
       
     	# find error 
     	[err, total] = p.update(model)
       
     	# run model for rotor speeds
     	raw_data = model.rotor_speeds(err, total)

     	# translate model 
     	model.translate_model(raw_data, p.dt)
     	mod_data = model.state.thetadots
            
     	#send yaw commands ***non yaw commands aka stabalization commands not considered) 
     	rt.talk(mod_data)


    rospy.loginfo('Program End Reached')
    

