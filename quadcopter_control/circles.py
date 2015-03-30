#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import time
import math

ang = 0

def callback(data):
    # data.pose[1] is the quadcopter
    ang = data.pose[1].orientation.z

def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('gazebo/model_states', ModelStates, callback)
    rospy.spin()

def talker(start_time):
    # start publisher & node
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    prevState = 0
    state = 1

    while not rospy.is_shutdown():
        elapsed = time.time() - start_time # time since program run

        if elapsed < 5:
            state = 1
        elif elapsed >= 5 and elapsed < 10:
            state = 2

        if True: # currently no conditional...need to send commands continuously
            if state == 1: # go straight up
                prevState = 1
                state = 2
                twist = Twist()
                twist.linear.z = 1
                pub.publish(twist)
                rate.sleep()
            elif state == 2: # trace circle
                prevState = 2
                twist = Twist()
                twist.linear.x = 1 # translate
                twist.angular.z = 36 # spin
                pub.publish(twist)
                rate.sleep()

if __name__ == '__main__':
    try:
        #listener()
        talker(time.time())
    except rospy.ROSInterruptException:
        pass