#!/usr/bin/env python
import rospy
import actionlib
import odrive_ros.msg
import time

def fdbk_cb(msg):
    print(msg)
    return

if __name__=="__main__":
    rospy.init_node("test_odrive_action")
    client = actionlib.SimpleActionClient("/name0/velocity",odrive_ros.msg.SetpointAction)
    client.wait_for_server()
    goal = odrive_ros.msg.SetpointGoal(setpoint=0, tolerance=0.25)
    client.send_goal(goal,feedback_cb=fdbk_cb)
    client.wait_for_result()
    print(client.get_result())
    time.sleep(1)
