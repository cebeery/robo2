# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 08:07:56 2015

@author: claire
Extended variation of 
https://github.com/gibiansky/experiments/blob/master/quadcopter/matlab/ 
"""

#import rospy
#from geometry_msgs.msg import Twist
import numpy as np
import math


class QuadrotorParameters:
    """Stores information relevant to a quadcopter player; ***static***"""
    
    def __init__(self):
        """ Initializes State object"""
        self.m = 0.5    # mass of quadcopter, kg
        self.g = 9.81   # gravitational constant, m^2/s
        self.k = 3e-6   # relates thrust to square of angular velocity (?)
        self.L = 0.25   # quadcopter arm length, m
        self.b = 1e-7   # drag coefficient (?)
        self.I = np.matrix([[5e-3, 0, 0], [0, 5e-3, 0], [0, 0, 5e-3]]) # inertia matrix



class PidController:
    """ Creates a PID Controller that references the player information"""
    
    def __init__(self, Kd=4, Kp=3, Ki=5.5):
        """ Initalizes PID Controller object with PID constants, gains, and Quadrotor instance"""
        self.quad = QuadcopterParameters()
        
        self.dt = 0.005 # timestep, ***static***
        self.proportional = np.zeros(3)
        self.integral = np.zeros(3)
        
        self.Kd = Kd
        self.Kp = Kp
        self.Ki = Ki
        
     # compute quadcopter thrusts from pid error
    def err2inputs(self, err, total):
        """ F"""
        # unpack error
        e1 = err[0]
        e2 = err[1]
        e3 = err[2]
 
        # unpack quadrotor parameters
        Ix = self.quad.I[0,0]
        Iy = self.quad.I[1,1]
        Iz = self.quad.I[2,2]
        k = self.quad.k
        L = self.quad.L
        b = self.quad.b

        # compute quadcopter rotor speed inputs
        rotor_speed = np.zeros(4)
        rotor_speed[0] = total/4 -(2 * b * e1 * Ix + e3 * Iz * k * L)/(4 * b * k * L)
        rotor_speed[1] = total/4 + e3 * Iz/(4 * b) - (e2 * Iy)/(2 * k * L)
        rotor_speed[2] = total/4 -(-2 * b * e1 * Ix + e3 * Iz * k * L)/(4 * b * k * L)
        rotor_speed[3] = total/4 + e3 * Iz/(4 * b) + (e2 * Iy)/(2 * k * L)

        return rotor_speed

    def update(self, thetadot):
        """ Performs PID update against setpoint [0,0,0], adjusts thetadot of 
        rotors, updates intregral states (no sensors)??"""
    
        # unpack quadrotor parameters
        g = self.quad.g
        m = self.quad.m
        k = self.quad.k
        L = self.quad.L
        b = self.quad.b
        #prevent intergral windup
        if max(self.integral) > 0.01:
            self.integral = np.zeros(3)

        # Compute total thrust. ***
        total = m * g / k / math.cos(self.proportional[0]) * math.cos(self.proportional[2])

        # Compute error against zero and adjust with PID 
        err = self.Kd * thetadot + self.Kp * self.proportional - self.Ki * self.integral
        
        # Compute rotor speeds.
        inputs = self.err2inputs(err, total)

        # Update controller state.
        self.proportional = self.proportional + self.dt * thetadot
        self.integral = self.integral + self.dt * self.proportional

        return raw_outputs

class ROSTalker:
    """ """ 
    
    def _init_(self):
        """ """
        #attributes
        self.subscriber_data = None
        self.rate = rospy.Rate(10) # 10hz\
        self.twist = Twist() #sets velocity commands to 0
        
        # start publisher & subscriber
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, callback)
        
        # initialize node
        rospy.init_node('talker', anonymous=True)
        
        # initialize twist commands, note: inital statement is unpublished    
        pub.publish(twist)
        

    def callback(self, msg):
        self.subscriber_data = msg.linear.x
        #rospy.loginfo(msg.linear.z)
        
    def rise(self):        
        # raise quadcopter
        self.rate.sleep()
        self.twist.linear.z = 1 
        self.pub.publish(twist)
        
        # rise time
        time.sleep(1)
         
        # stop rising
        self.rate.sleep()
        self.twist = Twist()
        self.pub.publish(twist)
        
    def talk(self):
        pass
        
        

if __name__ == "__main__":
    
    

    
    # test controller
    p = PidController()
    thetadot = np.array([1,2,3])
    
    output = p.update(thetadot)
    print(output)
    
    
    # create ROS node and functions
    rt = ROSTalker
    
    # raise quadcopter
    rt.rise() 
    
    #create quadcopter state tracker ***unused at current
    state = State()
    
    #create model traslator
    trans = ModelTranslation()
    
    #print controller
    if p == None:
        roslog("No controller; using built_in inputs")

    #behaviors run loop
    while not rospy.is_shutdown():
        
        # *** collect current data ***
        if rt.subscriber_data is not None:
            #rospy.loginfo(rt.subscriber_data)
            rt.subscriber_data = None
        
        # *** translate data *** ***does not look at real thetadot
        thetadot = thetadot 
        #*** does thetadot/omega need to be tracked or is it pullable from a gazebo topic
        
        # run model  *** PID control includes modle at current 
        # Get input from built-in input or controller.
        if p == None
            raw_output = input(t);  #***
        else:
            raw_output = p.update(thetadot);

        
        # *** translate model ***
        mod_output = trans.update(thetadot, raw_output)
            
        #send yaw commands ***non yaw commands aka stabalization commands not considered) 
        rt.rate.sleep() 
        rt.twist = Twist()
        rt.twist.angular.z = mod_output 
        rt.pub.publish(twist)

