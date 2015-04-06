#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
import time

# why doesn't this work?
def talker_2(start_time):
    # start publisher & node
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    pub.publish(Twist())

    if not rospy.is_shutdown():
        rospy.loginfo('first command')
        twist = Twist()
        twist.linear.z = 1 # go straight up
        pub.publish(twist)
        rate.sleep()

        time.sleep(5)
        rospy.loginfo('second command')

        twist.linear.z = 0 # stop going up
        pub.publish(twist)
        rate.sleep()


def talker(start_time):
    # start publisher & node
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    twist = Twist()
    twist.linear.z = 1 # go straight up
    pub.publish(Twist())

    prevState = 0
    state = 1

    while not rospy.is_shutdown():
        elapsed = time.time() - start_time # time since program run

        if elapsed < 5:
            state = 1
        elif elapsed >= 5 and elapsed < 10:
            state = 2

        if True:
            #prevState != state: # why doesn't this work?
            if state == 1:
                prevState = 1
                twist = Twist()
                twist.linear.z = 1 # go straight up
                pub.publish(twist)
                rate.sleep()
            elif state == 2:
                prevState = 2
                twist = Twist()
                twist.linear.z = 0 # stop going up
                pub.publish(twist)
                rate.sleep()

if __name__ == '__main__':
    try:
        talker(time.time())
    except rospy.ROSInterruptException:
        pass