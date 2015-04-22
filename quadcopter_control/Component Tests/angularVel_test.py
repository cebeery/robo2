#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import time

global orientation 


def callback(msg):
    global orientation
    #orientation = msg.pose[1].orientation
    orientation = msg
    #rospy.loginfo(orientation)


def talker(start_time):
    global orientation

    # subscriber and publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pub_1 = rospy.Publisher('gazebo/set_model_states', ModelStates, queue_size=10)
    sub = rospy.Subscriber('gazebo/model_states', ModelStates, callback)


    # initiate node
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # initialize, inital unpublished publish statement sent
    twist = Twist()
    pub.publish(twist)
    rate.sleep() 

    # rise to test position 
    twist.linear.z = 1 # go straight up
    pub.publish(twist)
    rospy.loginfo(orientation)
    rate.sleep()
    time.sleep(5)

    twist.linear.z = 0 # hover
    pub.publish(twist)
    rate.sleep()
    rospy.loginfo('Hover State Reached')
    twist = Twist() 
    time.sleep(2)


    # .....test.....#

    rospy.loginfo('Test linear:') 
    twist.linear.x = 5 
    pub.publish(twist)
    rate.sleep()
    rospy.loginfo(orientation)
    time.sleep(3)
    rospy.loginfo(orientation)

    twist.linear.x = 0 
    pub.publish(twist)
    rate.sleep())
    time.sleep(1)
    rospy.loginfo(orientation)

    # .....test.....#
	      
    rospy.loginfo('Program End State')


if __name__ == '__main__':
    try:
        talker(time.time())
    except rospy.ROSInterruptException:
        pass
