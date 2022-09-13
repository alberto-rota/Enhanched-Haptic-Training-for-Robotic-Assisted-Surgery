#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

topic = "/dvrk/console/teleop/set_scale"

def talker():   
    rospy.init_node('console_setscale', anonymous=True)
    pub = rospy.Publisher(topic, Float32, queue_size=10)
    rate = rospy.Rate(20) # 10hz

    SCALE = 0.5
    
    
    print("> Teleoperation scale initialized to", SCALE)
    
    # Must publish a few times to get recieved by subscribers
    for _ in range(10):
        pub.publish(SCALE)
        rate.sleep()
  
    # Empty loop: Scale is only initialized at 0.5, operator will have the chance to change it from the console
    # ! PUBLISH INSIIDE THE WHILE LOOP TO KEEP THE SCALE FIXED !
    while not rospy.is_shutdown():
        pass
        
        
if __name__ == '__main__':  
    try:
        talker()
    except rospy.ROSInterruptException:
        pass