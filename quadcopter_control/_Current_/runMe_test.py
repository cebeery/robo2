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
from model import State
       

if __name__ == "__main__":
    
    # create ROS node and functions
    rt = ROSTalker() 
   
    # raise quadcopter
    rt.rise(time.time())  
    
    #create model/state
    state = State()

    #create first half of dynamics, 'tuned parameters'
    model = RotorDynamicsModel(state)

    #create second half of dynamics, 'real parameters'
    translation = TranslationalController(state)
    
    #create PID controller for trajectories
    p = PidController()
    
    rospy.loginfo('Start Reached')
    
    
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
     	translation.translate_model(raw_data, p.dt)
     	mod_data = state.thetadots
            
     	#send yaw commands ***non yaw commands aka stabalization commands not considered) 
     	rt.talk(mod_data)


    rospy.loginfo('Program End Reached')
