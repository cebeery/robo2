#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


collect = {
    'px': [],
    'py': [],
    'pz': [],
    'ax': [],
    'ay': [],
    'az': []
}


def callback(data):
    # unpack values
    values = [data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z]
    keys = ['px', 'py', 'pz', 'ax', 'ay', 'az']

    # add values to collect by key
    for i in range(len(values)):
        collect[keys[i]].append(values[i])
    
    rospy.loginfo(values)


def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('/cmd_vel', Twist, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()