import numpy as np
import math

# test controller
def test():
    state = {
        'integral': np.zeros(3),
        'integral2': np.zeros(3),
        'm': 0.5,
        'g': 9.81,
        'k': 3e-6,
        'dt': 0.005,
        'L': 0.25,
        'b': 1e-7,
        'I': np.matrix([[5e-3, 0, 0], [0, 5e-3, 0], [0, 0, 5e-3]])
    }

    thetadot = np.array([1,2,3])
    [controlInput, state] = pid_controller(state, thetadot)
    return controlInput


# update controller with one thetadot step
def pid_controller(state, thetadot, Kd=4, Kp=3, Ki=5.5):
    # Prevent wind-up
    if max(state['integral2']) > 0.01:
        state['integral2'] = np.zeros(3)

    # Compute total thrust.
    total = state['m'] * state['g'] / state['k'] / math.cos(state['integral'][0]) * math.cos(state['integral'][2]);

    # Compute error and inputs.
    err = Kd * thetadot + Kp * state['integral'] - Ki * state['integral2']
    controlInput = err2inputs(state, err, total)

    # Update controller state.
    state['integral'] = state['integral'] + state['dt'] * thetadot
    state['integral2'] = state['integral2'] + state['dt'] * state['integral']

    return [controlInput, state]


# compute model inputs (rotor velocities) given error
def err2inputs(state, err, total):
    e1 = err[0]
    e2 = err[1]
    e3 = err[2]
    Ix = state['I'][0,0]
    Iy = state['I'][1,1]
    Iz = state['I'][2,2]
    k = state['k']
    L = state['L']
    b = state['b']

    inputs = np.zeros(4)
    inputs[0] = total/4 -(2 * b * e1 * Ix + e3 * Iz * k * L)/(4 * b * k * L)
    inputs[1] = total/4 + e3 * Iz/(4 * b) - (e2 * Iy)/(2 * k * L)
    inputs[2] = total/4 -(-2 * b * e1 * Ix + e3 * Iz * k * L)/(4 * b * k * L)
    inputs[3] = total/4 + e3 * Iz/(4 * b) + (e2 * Iy)/(2 * k * L)

    return inputs