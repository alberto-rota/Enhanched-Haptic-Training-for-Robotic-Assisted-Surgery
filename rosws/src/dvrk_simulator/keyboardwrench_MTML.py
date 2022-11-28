#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Wrench
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

topic = "/dvrk/MTML/set_wrench_body_safe"

RESOLUTION = 0.1

def talker():
    pub = rospy.Publisher(topic, Wrench, queue_size=10)
    rospy.init_node('wrench_keyboard', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    wrench = Wrench()
    
    print("> Waiting for keyboard input")
    print("W/S: Forward/Backwards - A/D: Left/Right - P/L: Up/Down\n")
    while not rospy.is_shutdown():
        settings = saveTerminalSettings()
        k = getKey(settings, 0.05)
        if k == 'w':
            wrench.force.x+=RESOLUTION
        elif k == 's':
            wrench.force.x-=RESOLUTION
        if k == 'a':
            wrench.force.y+=RESOLUTION
        elif k == 'd':
            wrench.force.y-=RESOLUTION
        if k == 'p':
            wrench.force.z+=RESOLUTION
        elif k == 'l':
            wrench.force.z-=RESOLUTION
            
        # rospy.loginfo(joints)
        pub.publish(wrench)
        rate.sleep()
  
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass