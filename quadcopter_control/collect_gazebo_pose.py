#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates

def callback(data):
    # data.pose[1] is the quadcopter
    rospy.loginfo(data.pose[1].position.x)


def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('gazebo/model_states', ModelStates, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()