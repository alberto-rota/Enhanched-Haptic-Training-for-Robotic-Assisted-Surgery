#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

from textual.app import App
from textual.widgets import Placeholder
from textual.widget import Widget
from rich.panel import Panel


jointsnow = 6

class Logger(Widget):
    def on_mount(self):
        self.set_interval(1, self.render)

    def render(self) -> Panel:
        return Panel(str(jointsnow))


class SimpleApp(App):

    async def on_mount(self) -> None:
        await self.view.dock(Logger(), edge="left", size=40)
        await self.view.dock(Placeholder(), Placeholder(), edge="top")


def callbackR(joints):
    print("reading")
    jointsnow = 5
    Logger.render(Logger())

topicR = "/dvrk/PSM1/state_joint_current"
rospy.init_node('console', anonymous=True)
subR = rospy.Subscriber(topicR, JointState, callbackR)
# subL = rospy.Subscriber(topicL, JointState, callbackL)
# pubOFF = rospy.Publisher(unpowertopic, Empty, queue_size=10)

# def wrench_safety_buffer():
rate = rospy.Rate(20) 
print("> [spring_green3]Joint Velocity Safety Virtual Relay Initialized ")

# pub.publish(wrench)
# rate.sleep()
SimpleApp.run(log="textual.log")
rospy.spin()
  
# if __name__ == '__main__':
#     try:
#        wrench_safety_buffer()
#     except rospy.ROSInterruptException:
#         pass
