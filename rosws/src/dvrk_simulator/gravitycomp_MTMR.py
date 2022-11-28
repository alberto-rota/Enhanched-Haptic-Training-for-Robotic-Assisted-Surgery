#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

topic_r = "/dvrk/MTMR/set_gravity_compensation"
# topic_l = "/dvrk/MTML/set_gravity_compensation"


def talker():
    rospy.init_node('MTM_gravity_compensation', anonymous=True)
    pubr = rospy.Publisher(topic_r, Bool, queue_size=10)
    # publ = rospy.Publisher(topic_l, Bool, queue_size=10)
    rate = rospy.Rate(20) # 10hz

    print("> Gravity compensation is ACTIVE on Right-MTM")
    # print("Gravity compensation is ACTIVE on MTML")
    
    while not rospy.is_shutdown():
    
        pubr.publish(True)
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