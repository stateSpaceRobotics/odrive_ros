#!/usr/bin/env python
import rospy
from odrive_ros.odrive_interface import ODrive_ROS
import pprint
if __name__=="__main__":
    rospy.init_node("odrive_node", log_level=rospy.DEBUG)
    odrive_node = ODrive_ROS("odrive_node")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        odrive_node.update()
        rate.sleep()
