# -*- coding: utf-8 -*-

import numpy as np
from quadrotor_utils import Model, PidController
        
def test():
    #create model and state
    model = Model()
    model.state.disturb() #disturb system for testing
    
    #create PID controller
    p = PidController()
    
    #----test----#
    
    #create empties for tracking
    errors  = np.zeros([20,3])
    ang_vel = np.zeros([20,3])

    for x in range(0, 20):
        # find error
        [err, total] = p.update(model)
        errors[x,:] = err.flatten()
       
        # run model for rotor speeds
        raw_data = model.rotor_speeds(err, total)

        # translate model 
        model.rotors2thetadot(raw_data, p.dt)
        mod_data = model.state.thetadots
        ang_vel[x,:] = mod_data.flatten()
    
    print('errors')
    print(errors)
    print('ang_vel')
    print(ang_vel)

if __name__ == "__main__":
    test()