#!/usr/bin/env python
import rospy
import actionlib
import odrive_ros
import odrive_ros.msg
import time
from std_srvs.srv import Trigger,TriggerResponse
from actionlib_msgs.msg import GoalStatus

def fdbk_cb(msg):
    print(msg)
    return

if __name__=="__main__":
    rospy.init_node("test_odrive_action")
    rospy.loginfo("waiting for calibrate service")
    rospy.wait_for_service("/name0/calibrate")
    rospy.wait_for_service("/name1/calibrate")
    rospy.loginfo("Calling calibrate service")
    rospy.loginfo(rospy.ServiceProxy("/name0/calibrate", Trigger)())
    rospy.loginfo(rospy.ServiceProxy("/name1/calibrate", Trigger)())
    # action
    rospy.loginfo("Waiting for action servers")
    client0 = actionlib.SimpleActionClient("/name0/velocity",odrive_ros.msg.SetpointAction)
    client1 = actionlib.SimpleActionClient("/name1/position",odrive_ros.msg.SetpointAction)
    client0.wait_for_server()
    client1.wait_for_server()
    
    client0.send_goal(
        goal=odrive_ros.msg.SetpointGoal(
            setpoint=50,
            tolerance=0.1
        ),
        feedback_cb=fdbk_cb
    )
    client0.wait_for_result()
    print(client0.get_state(), client0.get_result())
    time.sleep(1)
    client0.send_goal(
        goal=odrive_ros.msg.SetpointGoal(
            setpoint=0,
            tolerance=0.1
        ),
        feedback_cb=fdbk_cb
    )
    client0.wait_for_result()
    print(client0.get_state(), client0.get_result())
    
    client1.send_goal(
        goal=odrive_ros.msg.SetpointGoal(
            setpoint=100,
            tolerance=0.1
        ),
        feedback_cb=fdbk_cb
    )
    client1.wait_for_result()
    print(client1.get_state(), client1.get_result())
    time.sleep(1)
    client1.send_goal(
        goal=odrive_ros.msg.SetpointGoal(
            setpoint=0,
            tolerance=0.1
        ),
        feedback_cb=fdbk_cb
    )
    client1.wait_for_result()
    print(client0.get_state(), client1.get_result())
