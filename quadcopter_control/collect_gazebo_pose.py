#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates

def callback(data):
    rospy.loginfo(data.pose[1].position.x)


def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('gazebo/model_states', ModelStates, callback)
    rospy.spin()


if __name__ == '__main__':
    #plt.ion()
    #plt.plot([1.6,2.7])
    #plt.title('interactive test')
    listener()