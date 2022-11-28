#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Wrench
from std_msgs.msg import Empty
import math
from rich import print

inputtopicR = "/dvrk/MTMR/set_wrench_body_safe"
inputtopicL = "/dvrk/MTML/set_wrench_body_safe"
outputtopicR = "/dvrk/MTMR/set_wrench_body"
outputtopicL = "/dvrk/MTML/set_wrench_body"
unpowertopic = "/dvrk/console/power_off"

CUTOFF_FORCE = 3
UNPOWER_FORCE = 5
CUTOFF_TORQUE = 0.1
UNPOWER_TORQUE = 0.4



def sqmag(force):
    return force.x**2+force.y**2+force.z**2
def mag(force):
    return math.sqrt(sqmag(force))
def dir(force):
    magf = mag(force)
    force.x /= magf
    force.y /= magf
    force.z /= magf
    return force

def cutoff(force, limit):
    forcen = dir(force)
    force.x = forcen.x*limit
    force.y = forcen.y*limit
    force.z = forcen.z*limit
    return force


def bufferR(wrench):
    fwrench = wrench
    wrenchforcemag = sqmag(wrench.force)
    wrenchtorquemag = sqmag(wrench.torque)

    if wrenchforcemag >= UNPOWER_FORCE**2:
        print(f"[red] DANGER: Force on MTMR over unpowering limit: {math.sqrt(wrenchforcemag)} > {UNPOWER_FORCE} [N]  ",end="")
        print(f"[red] >>   Cutting power from all arms")
        pubOFF.publish(Empty())
        return

    elif wrenchforcemag >= CUTOFF_FORCE**2:
        fwrench.force = cutoff(wrench.force, CUTOFF_FORCE)
        print(f"[yellow] WARNING: Force on MTMR over cutoff limit: {math.sqrt(wrenchforcemag)} > {CUTOFF_FORCE} [N]  ")

    if wrenchtorquemag >= UNPOWER_TORQUE**2:
        print(f"[red] DANGER: Torque on MTMR over unpowering limit: {math.sqrt(wrenchtorquemag)} > {UNPOWER_TORQUE} [Nm]  ",end="")
        print(f"[red] >>   Cutting power from all arms")
        pubOFF.publish(Empty())
        return

    elif wrenchtorquemag >= CUTOFF_TORQUE**2:
        print(f"[yellow] WARNING: Torque on MTMR over cutoff limit: {math.sqrt(wrenchtorquemag)} > {CUTOFF_TORQUE} [Nm]  ")
        fwrench.torque = cutoff(wrench.torque, CUTOFF_TORQUE)

    pubR.publish(fwrench)

def bufferL(wrench):
    fwrench = wrench
    wrenchforcemag = sqmag(wrench.force)
    wrenchtorquemag = sqmag(wrench.torque)

    if wrenchforcemag >= UNPOWER_FORCE**2:
        print(f"[red] DANGER: Force on MTML over unpowering limit: {math.sqrt(wrenchforcemag)} > {UNPOWER_FORCE} [N]  ",end="")
        print(f"[red] >>   Cutting power from all arms")
        pubOFF.publish(Empty())
        pubR.publish(Wrench())
        return

    elif wrenchforcemag >= CUTOFF_FORCE**2:
        fwrench.force = cutoff(wrench.force, CUTOFF_FORCE)
        print(f"[yellow] WARNING: Force on MTML over cutoff limit: {math.sqrt(wrenchforcemag)} > {CUTOFF_FORCE} [N]  ")

    if wrenchtorquemag >= UNPOWER_TORQUE**2:
        print(f"[red] DANGER: Torque on MTML over unpowering limit: {math.sqrt(wrenchtorquemag)} > {UNPOWER_TORQUE} [Nm]  ",end="")
        print(f"[red] >>   Cutting power from all arms")
        pubOFF.publish(Empty())
        pubL.publish(Wrench())
        return

    elif wrenchtorquemag >= CUTOFF_TORQUE**2:
        print(f"[yellow] WARNING: Torque on MTML over cutoff limit: {math.sqrt(wrenchtorquemag)} > {CUTOFF_TORQUE} [Nm]  ")
        fwrench.torque = cutoff(wrench.torque, CUTOFF_TORQUE)

    pubL.publish(fwrench)

rospy.init_node('wrench_safety_buffer', anonymous=True)

subR = rospy.Subscriber(inputtopicR, Wrench, bufferR)
pubR = rospy.Publisher(outputtopicR, Wrench, queue_size=10)
subL = rospy.Subscriber(inputtopicL, Wrench, bufferL)
pubL = rospy.Publisher(outputtopicL, Wrench, queue_size=10)
pubOFF = rospy.Publisher(unpowertopic, Empty, queue_size=10)

def wrench_safety_buffer():
    rate = rospy.Rate(20) 
    print("[spring_green3]> Wrench Safety buffer initialized")
    
    # pub.publish(wrench)
    # rate.sleep()
    rospy.spin()
  
if __name__ == '__main__':
    try:
        wrench_safety_buffer()
    except rospy.ROSInterruptException:
        pass