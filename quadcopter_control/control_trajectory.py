#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import numpy as np
from ros_utils import RosNode
import trajectory_utils
import bisect

class ControlTrajectory(RosNode):
    def __init__(self, trajectory):
        self.keyframes = trajectory.keyframes
        self.desiredIndex = 0
        RosNode.__init__(self)

    def switch(self, elapsed):
        # self.training defaults to true -> in training mode already
        riseTime = 2

        # rise
        if (elapsed < riseTime):
            self.rate.sleep()
            self.twist.linear.z = 1
            self.pub.publish(self.twist)

        # train (currently, while controlling to flat)
        elif (self.model.training):
            # unpack keyframes
            times = self.keyframes['t']
            thetadots = self.keyframes['thdot']

            # index into keyframes
            trajectoryTime = elapsed - riseTime
            index = bisect.bisect_left(times, trajectoryTime)

            # set z-axis spin setpoint
            setpoint = np.zeros([3,1])
            rospy.loginfo(index)
            if index > len(times) - 1:
                index = len(times) - 1
            setpoint[2,0] = thetadots[index]/36
            rospy.loginfo(setpoint[2,0])
            rospy.loginfo('--')

            # compute angular velocities
            [err, total] = self.controller.update(self.model, setpoint)
            raw_data = self.model.rotor_speeds(err, total)
            self.model.rotors2thetadot(raw_data, self.controller.dt)

            # update learned parameters (currently dummy method)
            self.model.updateLearn(self.model.state.thetas, self.model.state.thetadots, None, None)

            # publish angular velocities
            self.twist = Twist()
            self.twist.linear.x = 1
            self.twist.angular.x = self.model.state.thetadots[0,0]
            self.twist.angular.y = self.model.state.thetadots[1,0]
            self.twist.angular.z = self.model.state.thetadots[2,0]
            self.pub.publish(self.twist)
            self.rate.sleep()

            if (index >= len(times)-1): # done with maneuver...replace with real metric
                self.model.training = False # exit training mode

        # out of training mode - now do complicated maneuvers
        else: 
            rospy.loginfo('Done training')
            self.twist = Twist() # WRITE COMPLEX MANEUVERS HERE
            self.pub.publish(self.twist)
            self.rate.sleep()


if __name__ == "__main__":
    traj = trajectory_utils.makeTrajectory(loadfile='circle')
    node = ControlTrajectory(traj)
    node.start()