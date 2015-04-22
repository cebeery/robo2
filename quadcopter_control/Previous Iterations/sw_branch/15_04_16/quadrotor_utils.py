import numpy as np
import rospy

''' FILE STRUCTURE '''
# -> class PidController
# -> class Model
#   -> has a QuadrotorParameters
#   -> has two States (learn & truth)
# -> class QuadrotorParameters
# -> class State


class PidController:
    """ Creates a PID Controller that references the player information"""
    
    def __init__(self, Kd=4, Kp=3, Ki=5.5):
        """ Initalizes PID Controller object with PID constants """
        
        self.dt = 0.005 # timestep, ***static***
        self.proportional = np.zeros([3,1])
        self.integral = np.zeros([3,1])
        
        self.Kd = Kd
        self.Kp = Kp
        self.Ki = Ki


    def update(self, model):
        """ Performs PID update against setpoint [0,0,0]
            Always use model's learn params
        """
    
        # unpack quadrotor parameters
        g = model.learn.g
        m = model.learn.m
        k = model.learn.k
        
        #prevent intergral windup
        if max(self.integral) > 0.01:
            self.integral = np.zeros([3,1])

        # Compute total thrust. ***
        total = m * g / k / np.cos(self.proportional[0,0]) * np.cos(self.proportional[2,0])

        # Compute error against [0,0,0] and adjust with PID 
        err = self.Kd * model.state.thetadots + self.Kp * self.proportional - self.Ki * self.integral

        # Update controller state.
        self.proportional = self.proportional + self.dt * model.state.thetadots
        self.integral = self.integral + self.dt * self.proportional

        return [err, total]


class Model:

    def __init__(self):
        """ learn = parameters to learn
            truth = parameters from Gazebo """ 

        self.state = State()
        self.learn = QuadrotorParameters()
        self.truth = QuadrotorParameters(truth=True)
        self.training = True


    def updateLearn(self, meas_thetas, meas_thetadots, des_thetas, des_thetadots):
        rospy.loginfo(meas_thetas)
        rospy.loginfo(meas_thetadots)
        rospy.loginfo(des_thetas)
        rospy.loginfo(des_thetadots)
        rospy.loginfo('-----------')

        self.learn = self.learn # do modification of learned parameters here        


    def rotor_speeds(self, err, total):
        """ compute rotor speeds necessary for control
            Always use learn params
        """

        # unpack error
        e1 = err[0,0]
        e2 = err[1,0]
        e3 = err[2,0]
 
        # unpack quadrotor parameters
        Ix = self.learn.I[0,0]
        Iy = self.learn.I[1,1]
        Iz = self.learn.I[2,2]
        k = self.learn.k
        L = self.learn.L
        b = self.learn.b

        # compute quadcopter rotor speed inputs
        rotor_speed = np.zeros([4,1])
        rotor_speed[0,0] = total/4 -(2 * b * e1 * Ix + e3 * Iz * k * L)/(4 * b * k * L)
        rotor_speed[1,0] = total/4 + e3 * Iz/(4 * b) - (e2 * Iy)/(2 * k * L)
        rotor_speed[2,0] = total/4 -(-2 * b * e1 * Ix + e3 * Iz * k * L)/(4 * b * k * L)
        rotor_speed[3,0] = total/4 + e3 * Iz/(4 * b) + (e2 * Iy)/(2 * k * L)

        return rotor_speed


    def rotors2thetadot(self, rotors, dt):
        """ translates rotor speeds into angular velocities
            Params unused 
        """

        omega = self.thetadot2omega()
        omegadot = self.angular_acceleration(rotors, omega)

        # Advance system state.
        omega = omega + dt * omegadot        
        self.omega2thetadot(omega)


    def angular_acceleration(self, rotors, omega):
        """ Compute angular acceleration in body frame 
            Use truth params when training & learn params when maneuvering
        """

        if self.training:
            PARAMS = self.truth
        else:
            PARAMS = self.learn

        tau = self.torques(rotors)
        inv_I = np.linalg.inv(PARAMS.I)
        
        cross = np.cross(omega.flatten(), np.dot(PARAMS.I, omega).flatten())
        omegadot =  np.dot(inv_I, tau - np.reshape(cross,[3,1]))
        return omegadot


    def thetadot2omega(self):
        """ convert derivatives of roll, pitch, yaw to omega
            Params unused

        ***what is omega***
        ***math came from?*** """
    
        W = self.findW()
        omega = np.dot(W,self.state.thetadots)
        
        return omega
        
        
    def omega2thetadot(self, omega):
        """ convert omega to derivatives of roll, pitch, yaw
            Params unused

        ***what is omega***
        ***math came from?*** """
    
        W = self.findW()
        inv_W = np.linalg.inv(W)
        self.state.thetadots = np.dot(inv_W, omega)
    

    def torques(self, rotors):
        """ Compute torques 
            Use truth params when training & learn params when maneuvering 
        """

        if self.training:
            PARAMS = self.truth
        else:
            PARAMS = self.learn

        tau = np.array([
        [PARAMS.L * PARAMS.k * (rotors[0,0] - rotors[2,0])],
        [PARAMS.L * PARAMS.k * (rotors[1,0] - rotors[3,0])],
        [PARAMS.b * (rotors[0,0] - rotors[1,0] + rotors[2,0] - rotors[3,0])]
        ])

        return tau  


    def findW(self):
        """ Params unused

        ???? what is w ???
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


class QuadrotorParameters():
    """Stores information relevant to a quadcopter player; ***static***"""
    
    def __init__(self, truth=False):
        """ Initializes State object"""
        self.g = 9.81 # gravitational constant, m^2/s

        # TODO: get real values from Gazebo model
        if truth:
            self.m = 0.5    # mass of quadcopter, kg
            self.k = 3e-6   # relates thrust to square of angular velocity (?)
            self.L = 0.25   # quadcopter arm length, m
            self.b = 1e-7   # drag coefficient (?)
            self.I = np.array([[5e-3, 0, 0], [0, 5e-3, 0], [0, 0, 5e-3]]) # inertia matrix
        
        # initial values for training
        else:
            self.m = 0.5    # mass of quadcopter, kg
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
