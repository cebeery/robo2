#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import inspect, os

keys = ['lx', 'ly', 'lz', 'ax', 'ay', 'az']
path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) #


def callback(data):
    values = [data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z]

    for i in range(len(values)):
        with open(path+'/txt/'+keys[i]+'.txt', 'a') as f:
            f.write(str(values[i])+',')
    
    rospy.loginfo(values)


def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('/cmd_vel', Twist, callback)
    rospy.spin()


def clearTxt():
    for key in keys:
        name = path+'/txt/'+key+'.txt'
        if os.path.isfile(name):
            with open(name, 'wb') as f:
                f.write('')


if __name__ == '__main__':
    clearTxt()
    listener()