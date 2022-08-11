#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import keyboard
import tty
import termios
import sys
from select import select

def getKey(settings, timeout):

    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

YAW = 0
PITCH = 1
INSERTION = 2
WRIST_ROLL = 3
WRIST_PITCH = 4
WRIST_YAW = 5

RESOLUTION_Y = 0.01
RESOLUTION_P = 0.005
RESOLUTION_I = 0.001

def talker():
    pinched = False;
    pub = rospy.Publisher("/dvrk/PSM2/state_joint_current", JointState, queue_size=10)
    pub2 = rospy.Publisher("/dvrk/PSM2/state_jaw_current", JointState, queue_size=10)

    rospy.init_node('keyboard_teleop', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    joints = JointState()
    joints.name = ['outer_yaw','outer_pitch','outer_insertion','outer_roll','outer_wrist_pitch','outer_wrist_yaw']
    joints.position = [0,0,0,0,0,0]

    jaw = JointState()
    jaw.name = ['jaw']
    jaw.position = [0]

    print("> Waiting for keyboard input")
    print("W/S: Pitch - A/D: Yaw - P/L: Insertion - M: Toggle Pinch\n")
    while not rospy.is_shutdown():
        settings = saveTerminalSettings()
        k = getKey(settings, 0.05)
        if k == 'w':
            joints.position[PITCH] += RESOLUTION_P
        elif k == 's':
            joints.position[PITCH] -= RESOLUTION_P
        if k == 'a':
            joints.position[YAW] += RESOLUTION_Y
        elif k == 'd':
            joints.position[YAW] -= RESOLUTION_Y
        if k == 'p':
            joints.position[INSERTION] += RESOLUTION_I
        elif k == 'l':
            joints.position[INSERTION] -= RESOLUTION_I
        if k == 'm':
            pinched = not pinched
        # rospy.loginfo(joints)
        pub.publish(joints)
        if pinched: jaw.position[0] = -2 
        else: jaw.position[0] = 1 
        pub2.publish(jaw)
        rate.sleep()
  
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass