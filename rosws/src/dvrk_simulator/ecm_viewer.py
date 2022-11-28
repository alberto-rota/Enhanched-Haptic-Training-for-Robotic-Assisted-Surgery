#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from rich import print

feedR = "/endoscope/raw/right/image_raw/compressed"
feedL = "/endoscope/raw/left/image_raw/compressed"

def imgR_recieved(data):
    print(data)

rospy.init_node('endoscope_viewer', anonymous=True)

subR = rospy.Subscriber(feedR, CompressedImage, imgR_recieved)
# pubR = rospy.Publisher(feedL, CompressedImage, imgL_recieved)

def main():
    rate = rospy.Rate(20) 
    
    rospy.spin()
  
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass