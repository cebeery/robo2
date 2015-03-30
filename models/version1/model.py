import numpy as np
import math

class State:
    def __init__(self):
        self.integral = np.zeros(3)
        self.integral2 = np.zeros(3)
        self.m = 0.5
        self.g = 9.81
        self.k = 3e-6
        self.dt = 0.005
        self.L = 0.25
        self.b = 1e-7
        self.I = np.matrix([[5e-3, 0, 0], [0, 5e-3, 0], [0, 0, 5e-3]])


class PidController:
    def __init__(self, Kd=4, Kp=3, Ki=5.5):
        self.state = State()
        self.Kd = Kd
        self.Kp = Kp
        self.Ki = Ki

    def control(self, thetadot):
        if max(self.state.integral2) > 0.01:
            self.state.integral2 = np.zeros(3)

        # Compute total thrust.
        total = self.state.m * self.state.g / self.state.k / math.cos(self.state.integral[0]) * math.cos(self.state.integral[2])

        # Compute error.
        err = self.Kd * thetadot + self.Kp * self.state.integral - self.Ki * self.state.integral2
        
        # Compute inputs.
        inputs = self.err2inputs(err, total)

        # Update controller state.
        self.state.integral = self.state.integral + self.state.dt * thetadot
        self.state.integral2 = self.state.integral2 + self.state.dt * self.state.integral

        return inputs

    def err2inputs(self, err, total):
        e1 = err[0]
        e2 = err[1]
        e3 = err[2]
        Ix = 0
        Iy = 0
        Iz = 0
        Ix = self.state.I[0,0]
        Iy = self.state.I[1,1]
        Iz = self.state.I[2,2]
        k = self.state.k
        L = self.state.L
        b = self.state.b

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