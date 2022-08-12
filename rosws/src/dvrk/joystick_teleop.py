#!/usr/bin/env python

import rospy
# from sensor_msgs.msg import JointState
import serial
import serial.tools.list_ports

topic = "/dvrk/PSM1/state_joint_current"

YAW = 0
PITCH = 1
INSERTION = 2
WRIST_ROLL = 3
WRIST_PITCH = 4
WRIST_YAW = 5

RESOLUTION_Y = 0.01
RESOLUTION_P = 0.005
RESOLUTION_I = 0.001

DEVICE_NAME = "USB-SERIAL CH340"
BAUD = 9600

PORT = "COM5"
s = serial.Serial(PORT,BAUD);
print(s)
ports = serial.tools.list_ports.comports(include_links=False)
for p in ports: 
    if p.description == DEVICE_NAME:
        PORT = p.device

def talker():
    pub = rospy.Publisher(topic, JointState, queue_size=10)
    rospy.init_node('keyboard_teleop', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    joints = JointState()
    joints.name = ['outer_yaw','outer_pitch','outer_insertion','outer_roll','outer_wrist_pitch','outer_wrist_yaw']
    joints.position = [0,0,0,0,0,0]
    
    while not rospy.is_shutdown():
        print(s.readline())
        k = ""
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
            
        # rospy.loginfo(joints)
        pub.publish(joints)
        rate.sleep()
    pass
  
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass