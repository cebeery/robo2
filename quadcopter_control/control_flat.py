#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import numpy as np
from ros_utils import RosNode

class ControlFlat(RosNode):
    def sub_callback(self, data):
        # set theta
        np_orientation = np.zeros([3,1])
        orientation = data.pose[1].orientation
        np_orientation[0,0] = orientation.x
        np_orientation[1,0] = orientation.y
        np_orientation[2,0] = orientation.z
        self.model.state.thetas = np_orientation

    def switch(self, elapsed):
        disturbed = False

        # self.training defaults to true -> in training mode already

        # rise for 2 seconds
        if (elapsed < 2):
            self.rate.sleep()
            self.twist.linear.z = 1
            self.pub.publish(self.twist)

        # then disturb once and control to flat
        elif (elapsed < 10):
            if not disturbed:
                self.model.state.disturb()
                disturbed = True

            # compute angular velocities
            [err, total] = self.controller.update(self.model)
            raw_data = self.model.rotor_speeds(err, total)
            self.model.rotors2thetadot(raw_data, self.controller.dt)
            rospy.loginfo(self.model.state.thetadots)

            # publish angular velocities
            self.twist = Twist()
            self.twist.angular.x = self.model.state.thetadots[0,0]
            self.twist.angular.y = self.model.state.thetadots[1,0]
            self.twist.angular.z = self.model.state.thetadots[2,0]
            self.pub.publish(self.twist)
            self.rate.sleep()

        # exit training mode - now do complicated maneuvers
        else: 
            self.training = False # exit training mode
            rospy.loginfo('Done training')
            self.twist = Twist() # WRITE COMPLEX MANEUVERS HERE
            self.pub.publish(self.twist)
            self.rate.sleep()


if __name__ == "__main__":
    node = ControlFlat()
    node.start()