import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import time
import numpy as np
from quadrotor_utils import Model, PidController

class RosNode:
    # BOILERPLATE - LEAVE AS IS
    def __init__(self):
        ''' create node w/ publisher & subscriber '''
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('gazebo/model_states', ModelStates, self.sub_callback)
        rospy.init_node('handler', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
        self.model = Model()
        self.controller = PidController()
        self.twist = Twist()

    def start(self):
        ''' call to start the node '''
        self.start_time = time.time()

        while not rospy.is_shutdown():
            elapsed = time.time() - self.start_time
            self.switch(elapsed)

    def sub_callback(self, data):
        ''' subscriber callback '''
        # set theta
        np_orientation = np.zeros([3,1])
        orientation = data.pose[1].orientation
        np_orientation[0,0] = orientation.x
        np_orientation[1,0] = orientation.y
        np_orientation[2,0] = orientation.z
        self.model.state.thetas = np_orientation

    # CHILD CLASS SHOULD OVERRIDE
    def switch(self, elapsed):
        ''' define publishing/subscribing behavior based on time '''
        pass