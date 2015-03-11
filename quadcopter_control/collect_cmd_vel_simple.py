#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

#import matplotlib.pyplot as plt

collect = {
    'px': [0]*100,
    'py': [0]*100,
    'pz': [0]*100,
    'ax': [0]*100,
    'ay': [0]*100,
    'az': [0]*100    
}

def callback(data):
    values = [data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z]
    keys = ['px', 'py', 'pz', 'ax', 'ay', 'az']

    for i in range(len(values)):
        collect[keys[i]].append(values[i])
    
    rospy.loginfo(values)


def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('/cmd_vel', Twist, callback)
    rospy.spin()


if __name__ == '__main__':
    #plt.ion()
    #plt.plot([1.6,2.7])
    #plt.title('interactive test')
    listener()