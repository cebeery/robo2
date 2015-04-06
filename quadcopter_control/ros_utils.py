import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import time
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

    def rise(self):
        ''' move up for 2 seconds & stop for 2 '''
        #rise_start = time.time()

        #elapsed = time.time() - rise_start
        # raise quadcopter
        self.rate.sleep()
        self.twist.linear.z = 1
        self.pub.publish(self.twist)
        
        # rise time
        time.sleep(1)
         
        # stop rising
        self.rate.sleep()
        self.twist = Twist()
        self.pub.publish(self.twist)

    def start(self):
        ''' call to start the node '''
        self.start_time = time.time()

        while not rospy.is_shutdown():
            elapsed = time.time() - self.start_time
            self.switch(elapsed)

    # CHILD CLASS SHOULD OVERRIDE
    def sub_callback(self, data):
        ''' subscriber callback '''
        # note: data.pose[1] is the quadcopter
        pass

    def switch(self, elapsed):
        ''' define publishing/subscribing behavior based on time '''
        pass