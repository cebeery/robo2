#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from ros_utils import RosNode

class CircleDemo(RosNode):
    def sub_callback(self, data):
        rospy.loginfo(data.pose[1].orientation.z)

    def switch(self, elapsed):
        if elapsed < 5: # go straight up
            twist = Twist()
            twist.linear.z = 1
            self.pub.publish(twist)
            self.rate.sleep()
        elif elapsed >= 5: # trace circle
            twist = Twist()
            twist.linear.x = 1 # translate
            twist.angular.z = 36 # spin
            self.pub.publish(twist)
            self.rate.sleep()

if __name__ == "__main__":
    node = CircleDemo()
    node.start()