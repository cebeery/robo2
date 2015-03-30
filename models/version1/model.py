# Ported to Python from https://github.com/gibiansky/experiments/blob/master/quadcopter/matlab/controller.m

import numpy as np
import math

# stores information relevant to quadcopter model
class State:
    def __init__(self):
        self.proportional = np.zeros(3)
        self.integral = np.zeros(3)
        self.m = 0.5 # mass of quadcopter, kg
        self.g = 9.81 # gravitational constant, m^2/s
        self.k = 3e-6 # relates thrust to square of angular velocity (?)
        self.dt = 0.005 # timestep
        self.L = 0.25 # quadcopter arm length, m
        self.b = 1e-7 # drag coefficient (?)
        self.I = np.matrix([[5e-3, 0, 0], [0, 5e-3, 0], [0, 0, 5e-3]]) # inertia matrix


# stores a State() instance and derivative/proportional/integral constants
# performs pid loop updates w/ process variable thetadot
class PidController:
    def __init__(self, Kd=4, Kp=3, Ki=5.5):
        self.state = State()
        self.Kd = Kd
        self.Kp = Kp
        self.Ki = Ki

    # perform a pid update w/ process variable thetadot & setpoint [0,0,0]
    def control(self, thetadot):
        if max(self.state.integral) > 0.01:
            self.state.integral = np.zeros(3)

        # Compute total thrust.
        total = self.state.m * self.state.g / self.state.k / math.cos(self.state.proportional[0]) * math.cos(self.state.proportional[2])

        # Compute error.
        err = self.Kd * thetadot + self.Kp * self.state.proportional - self.Ki * self.state.integral
        
        # Compute inputs.
        inputs = self.err2inputs(err, total)

        # Update controller state.
        self.state.proportional = self.state.proportional + self.state.dt * thetadot
        self.state.integral = self.state.integral + self.state.dt * self.state.proportional

        return inputs

    # compute quadcopter thrusts from pid error
    def err2inputs(self, err, total):
        # unpack error
        e1 = err[0]
        e2 = err[1]
        e3 = err[2]
 
        # unpack state
        Ix = self.state.I[0,0]
        Iy = self.state.I[1,1]
        Iz = self.state.I[2,2]
        k = self.state.k
        L = self.state.L
        b = self.state.b

        # compute quadcopter inputs
        inputs = np.zeros(4)
        inputs[0] = total/4 -(2 * b * e1 * Ix + e3 * Iz * k * L)/(4 * b * k * L)
        inputs[1] = total/4 + e3 * Iz/(4 * b) - (e2 * Iy)/(2 * k * L)
        inputs[2] = total/4 -(-2 * b * e1 * Ix + e3 * Iz * k * L)/(4 * b * k * L)
        inputs[3] = total/4 + e3 * Iz/(4 * b) + (e2 * Iy)/(2 * k * L)

        return inputs


# test controller
def test():
    p = PidController()
    thetadot = np.array([1,2,3])
    p.control(thetadot)