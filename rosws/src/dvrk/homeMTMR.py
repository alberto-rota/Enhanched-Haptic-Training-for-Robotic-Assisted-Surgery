#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped


topic = "/dvrk/MTMR/set_position_goal_cartesian"

def talker():
    pub = rospy.Publisher(topic, PoseStamped, queue_size=10)
    rospy.init_node('MTMR_homer', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    homepose = PoseStamped()
    homepose.pose.position.x = -0.11313254012880615
    homepose.pose.position.y = 0.011907013132151256
    homepose.pose.position.z = -0.25395653788167755
    homepose.pose.orientation.x = 0.57768346242738
    homepose.pose.orientation.y = 0.803463427648716
    homepose.pose.orientation.z = 0.10037052635040779
    homepose.pose.orientation.w = 0.10321867616392197
    
    
    # while not rospy.is_shutdown():
            
    rospy.loginfo(homepose)
    pub.publish(homepose)
        # rate.sleep()
  
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass