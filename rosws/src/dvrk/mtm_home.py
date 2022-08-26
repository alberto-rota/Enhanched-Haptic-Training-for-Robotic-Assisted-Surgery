#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

state_topic = "/dvrk/MTMR/set_robot_state"
joints_topic = "/dvrk/MTMR/set_position_joint"

# Set STATE to DVRK_POSITION_GOAL_JOINT 
rospy.init_node('MTMR_home', anonymous=True)
state_pub = rospy.Publisher(state_topic, String, queue_size=10)
state_pub.publish("DVRK_POSITION_GOAL_JOINT")

# Send JOINT POSITION GOAL

# If GOALREACHED, go back to EFFORT STATE


