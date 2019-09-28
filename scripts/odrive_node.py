#!/usr/bin/env python
import rospy
import odrive_interface

if __name__=="__main__":
    rospy.init_node("odrive_node")
    odrive_node = odrive_interface.ODrive_Interface("odrive_node")
    rospy.spin()
