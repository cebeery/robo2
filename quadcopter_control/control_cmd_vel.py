#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
import time

def talker(start_time):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    prevState = 0
    state = 1
    while not rospy.is_shutdown():
        elapsed = time.clock() - start_time

        if elapsed < 5:
            state = 1
        elif elapsed >= 5 and elapsed < 10:
            state = 2

        #if prevState != state:
        if True:
            if state == 1:
                prevState = 1
                twist = Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 1
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0
                #rospy.loginfo(twist)
                pub.publish(twist)
                rate.sleep()
                #rospy.loginfo(elapsed)
            elif state == 2:
                prevState = 2
                twist = Twist()
                twist.linear.z = 0
                #rospy.loginfo(twist)
                pub.publish(twist)
                rate.sleep()
                #rospy.loginfo(elapsed)

if __name__ == '__main__':
    try:
        talker(time.clock())
    except rospy.ROSInterruptException:
        pass