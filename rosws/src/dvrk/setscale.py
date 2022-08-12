#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

topic_r = "/dvrk/console/teleop/set_scale"
# topic_l = "/dvrk/MTML/set_gravity_compensation"


def talker():   
    rospy.init_node('console_setscale', anonymous=True)
    pubr = rospy.Publisher(topic_r, Float32, queue_size=10)
    # publ = rospy.Publisher(topic_l, Bool, queue_size=10)
    rate = rospy.Rate(20) # 10hz

    SCALE = 0.2
    
    print("> Teleoperation scale set to", SCALE)
    # print("Gravity compensation is ACTIVE on MTML")
    
    while not rospy.is_shutdown():
    
        pubr.publish(SCALE)
        # rospy.loginfo(True)
        # publ.publish(True)
        # rospy.loginfo(True)
        rate.sleep()
    pass
  
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass