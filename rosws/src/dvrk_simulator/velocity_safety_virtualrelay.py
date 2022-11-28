#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
import math
from rich import print

topicR = "/dvrk/MTMR/state_joint_current"
topicL = "/dvrk/MTML/state_joint_current"
unpowertopic = "/dvrk/console/power_off"

MAX_VELOCITY = [4,4,4,8,12,8,24]


def callbackR(joints):
    jvels = joints.velocity
    for i,jv in enumerate(jvels):
        if jv > MAX_VELOCITY[i]:
            pubOFF.publish(Empty())
            print(f"[red]DETECTED HIGH SPEED ON MTMR-Joint{i+1} - Opening Virtual Relay   ",end="")
            print(f"[yellow] MTMR-Joint{i+1} speed = {jv} > {MAX_VELOCITY[i]} \[deg/s]")
            print(f"[red]>>   Cutting power from all arms")
            return

def callbackL(joints):    
    jvels = joints.velocity
    for i,jv in enumerate(jvels):
        if jv > MAX_VELOCITY[i]:
            pubOFF.publish(Empty())
            print(f"[red]DETECTED HIGH SPEED ON MTML-Joint{i+1} - Opening Virtual Relay   ",end="")
            print(f"[yellow] MTML-Joint{i+1} speed = {jv} > {MAX_VELOCITY[i]} \[deg/s]")
            print(f"[red]>>   Cutting power from all arms")
            return

rospy.init_node('velocity_safety_virtualrelay', anonymous=True)

subR = rospy.Subscriber(topicR, JointState, callbackR)
subL = rospy.Subscriber(topicL, JointState, callbackL)
pubOFF = rospy.Publisher(unpowertopic, Empty, queue_size=10)

def wrench_safety_buffer():
    rate = rospy.Rate(20) 
    print("[spring_green3]> Joint Velocity Safety Virtual Relay Initialized ")
    
    # pub.publish(wrench)
    # rate.sleep()
    rospy.spin()
  
if __name__ == '__main__':
    try:
        wrench_safety_buffer()
    except rospy.ROSInterruptException:
        pass