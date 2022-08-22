#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from rich import print

topic = "/dvrk/MTMR/set_wrench_body_orientation_absolute"

def talker():
    rospy.init_node('set_wrench_frame', anonymous=True)
    pub = rospy.Publisher(topic, Bool, queue_size=10)
    # publ = rospy.Publisher(topic_l, Bool, queue_size=10)
    rate = rospy.Rate(20) # 10hz

    print("[orange3]> Right-MTM RF set to BASE")
    
    while not rospy.is_shutdown():
        pub.publish(True)
        # rospy.loginfo(True)
        rate.sleep()
    pass
  
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass