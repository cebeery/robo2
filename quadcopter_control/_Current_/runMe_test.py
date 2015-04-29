#!/usr/bin/env python

"""
@author: claire

for spyder--># -*- coding: utf-8 -*-
for ROS----->#!/usr/bin/env python
"""

import rospy
from geometry_msgs.msg import Twist
import numpy as np
import time

from controllers import *
from talk_controllers import *
from model import Model
       

if __name__ == "__main__":
    
    # create ROS node and functions
    rt = ROSTalker() 
   
    # raise quadcopter
    rt.rise()  
    
    #create model/state
    model = Model()
    rospy.loginfo('Model Created')
    #model.disturb()
    #rospy.loginfo('Model Disturbed')

    #show path
#    times = model.trajectory.keyframes['t']
#    d_thetadots = model.trajectory.keyframes['thdot']
#    rospy.loginfo('times')
#    rospy.loginfo(times)
#    rospy.loginfo('desired velocities')
#    rospy.loginfo(d_thetadots)

    #create first half of dynamics, 'tuned parameters'
    controller = RotorDynamicsModel(model)
    rospy.loginfo('Controller Created')

    #create second half of dynamics, 'real parameters'
    translater = TranslationalController()
    rospy.loginfo('Translational Controller Created')
    
    #create PID controller for trajectories
    p = PidController()
    rospy.loginfo('Velocity PID Created')
    
    rospy.loginfo('Start Reached')
    start_time = time.time()

    running = True
    
    #behaviors run loop
    while not rospy.is_shutdown() and running:
       
     	# collect current data 
        elapsed_time = time.time() - start_time
     	thetas = rt.subscriber_data 
       
     	# update model/translate data 
     	model.update(thetas)
        [setpoint, running] = model.key_vel(elapsed_time)
       
     	# find error from desired velocity
     	[err, total] = p.update(model, setpoint) 
       
     	# find rotor speeds
     	raw_data = controller.rotor_speeds(err, total, model)

     	# translate model 
     	mod_data = translater.translate_model(raw_data, model.thetadots, model.thetas, p.dt)
         
     	#send yaw commands
     	rt.talk(mod_data)


    rt.stop()
    rospy.loginfo('Program End Reached')
