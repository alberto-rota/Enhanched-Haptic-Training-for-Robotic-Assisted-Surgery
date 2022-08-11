#!/usr/bin/env python 
import numpy as np
from matplotlib import pyplot as plt
import rospy
from geometry_msgs.msg import Wrench
import math

def plot_x(msg):
    global counter
    if counter % 1 == 0:
        # stamp = msg.header.stamp
        # time = stamp.secs + stamp.nsecs * 1e-9
        # print(msg.force.x);
        f = math.sqrt(msg.force.x**2+msg.force.z**2+msg.force.y**2)
        plt.plot(counter, f,'.b')
        # plt.axis("equal")
        plt.xlim([counter-100, counter])
        plt.draw()
        plt.pause(0.00000000001)

    counter += 1

if __name__ == '__main__':
    counter = 0

    rospy.init_node("plotter")
    rospy.Subscriber("/dvrk/MTMR/set_wrench_body", Wrench, plot_x)
    # plt.ion()
    plt.show()
    rospy.spin()