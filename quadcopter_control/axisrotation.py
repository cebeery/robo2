import rospy
from geometry_msgs.msg import Twist
import time
import math


def talker(start_time):
    # start publisher & node
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    state = 1

    while not rospy.is_shutdown():
        elapsed = time.time() - start_time # time since program run

        if elapsed < 5:
            state = 1
        elif elapsed >= 5 and elapsed < 10:
            state = 2
        elif elapsed >= 10 and elapsed <15:
            state = 3

        if True: # currently no conditional...need to send commands continuously
            if state == 1: # X axis
                twist = Twist()
                twist.angular.x = 30 # spin in x
                pub.publish(twist)
                rate.sleep()
            elif state == 2: # Y axis
                twist = Twist()
                twist.angular.y = 30 # spin in y
                pub.publish(twist)
                rate.sleep()
            elif state == 3: # Z axis
                twist = Twist()
                twist.angular.z = 30 # spin in z
                pub.publish(twist)
                rate.sleep()

if __name__ == '__main__':
    try:
        talker(time.time())
    except rospy.ROSInterruptException:
        pass