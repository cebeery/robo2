# DOES NOT WORK YET

#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

import matplotlib
matplotlib.use('GTKAgg') # do this before importing pylab
import matplotlib.pyplot as plt

import gobject
import threading
import time

collect = {
    'px': [0]*100,
    'py': [0]*100,
    'pz': [0]*100,
    'ax': [0]*100,
    'ay': [0]*100,
    'az': [0]*100    
}

mutex = threading.Lock()
queue = list()
figure = plt.figure()
axes = figure.add_subplot(111)

class Spinner(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.daemon = True
    def run(self):
        rospy.spin()

def g_idle():
    global mutex, queue, figure, axes
    mutex.acquire()
    for segment in queue:
        rospy.loginfo('hi')
        axes.plot(data.linear)
        figure.suptitle('measured position')
        figure.canvas.draw()
    queue = list()
    mutex.release()
    time.sleep(0.1)

def callback(data):
    global mutex, queue, figure, axes
    values = [data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z]
    keys = ['px', 'py', 'pz', 'ax', 'ay', 'az']

    for i in range(len(values)):
        collect[keys[i]].append(values[i])
    
    rospy.loginfo(values)

    mutex.acquire()
    queue.append(data)
    mutex.release()

if __name__ == '__main__':
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('/cmd_vel', Twist, callback)
    spinner = Spinner()
    spinner.start()
    gobject.idle_add(g_idle)
    plt.show()