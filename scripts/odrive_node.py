#!/usr/bin/env python
import rospy
from odrive_ros.odrive_interface import ODrive_ROS
import pprint
if __name__=="__main__":
    rospy.init_node("odrive_node")
    odrive_node = ODrive_ROS("odrive_node")
    rospy.spin()
