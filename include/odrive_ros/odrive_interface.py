import math
import time

import odrive
from odrive.enums import *

import rospy
import actionlib
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger,TriggerResponse
from std_msgs.msg import Header

import odrive_ros.msg

class ODrive_ROS:
    class Axis_Action_Server:
        def __init__(self,axis, name, ctrl_type, cpr):
            self._axis = axis
            self._name = name
            self._type = ctrl_type
            self._cpr = cpr
            if self._type == "position":
                cb = self.pos_cb
            elif self._type == "velocity":
                cb = self.vel_cb
            
            self._as = actionlib.SimpleActionServer(
                "{}/{}".format(axis_dict["name"],axis_dict["type"]), # TODO Fix
                odrive_ros.msg.SetpointAction,
                execute_cb=cb,
                auto_start=False
                )
            self._as.start()

        def pos_cb(self, goal):
            self._axis.controller.pos_setpoint = goal.setpoint * self._cpr
            rospy.logdebug("Received position goal on {}: {}".format(self._name,self._axis.controller.pos_setpoint))
            rate = rospy.Rate(25)

            # supply feedback until to setpoint
            while abs(goal.setpoint-self._axis.encoder.pos_estimate/self._cpr) > goal.tolerance:
                # preempt check; if preempted exit
                if self._as.is_preempt_requested():
                    rospy.loginfo("{}/{} has been preempted!".format(self._name,self._type))
                    # motor should stop where it is
                    self._axis.controller.pos_setpoint = self._axis.encoder.pos_estimate
                    self._as.set_preempted()
                    return

                self._as.publish_feedback(
                    odrive_ros.msg.SetpointActionFeedback(
                        current_pos=self._axis.encoder.pos_estimate*self._cpr,
                        current_vel=self._axis.encoder.vel_estimate*self._cpr
                    )
                )
                rate.sleep()
            # success hit
            self._as.set_succeeded(
                odrive_ros.msg.SetpointActionResult(
                    final=self._axis.encoder.pos_estimate*self._cpr
                )
            )
            return

        def vel_cb(self, goal):
            self._axis.controller.vel_setpoint = goal.setpoint * self._cpr
            rospy.logdebug("Received velocity goal on {}: {}".format(self._name,self._axis.controller.vel_setpoint))
            rate = rospy.Rate(25)

            # supply feedback until to setpoint
            while abs(goal.setpoint-self._axis.encoder.vel_estimate/self._cpr) > goal.tolerance:
                # preempt check; if preempted exit
                if self._as.is_preempt_requested():
                    rospy.loginfo("{}/{} has been preempted!".format(self._name,self._type))
                    # motor should stop where it is
                    self._axis.controller.vel_setpoint = self._axis.encoder.vel_estimate
                    self._as.set_preempted()
                    return

                self._as.publish_feedback(
                    odrive_ros.msg.SetpointActionFeedback(
                        current_pos=self._axis.encoder.pos_estimate*self._cpr,
                        current_vel=self._axis.encoder.vel_estimate*self._cpr
                    )
                )
                rate.sleep()
            # success hit
            self._as.set_succeeded(
                odrive_ros.msg.SetpointActionResult(
                    final=self._axis.encoder.vel_estimate*self._cpr
                )
            )
            return

    def __init__(self,rosparam_name):
        self.name = rosparam_name
        # load parameters
        self.load_params()
        # connect
        self.connect()

        self.create_axis(self.params["axis0"]["name"],self.params["axis0"])
        self.create_axis(self.params["axis1"]["name"],self.params["axis1"])
        if self.params["twist"]["enable"]:
            self.sub = rospy.Subscriber(self.name, Twist, self.twist_cb)

    def load_params(self):
        """
        Loads ROS parameters using parameter name.
        """
        if not rospy.has_param(self.name):
            rospy.logerr("Parameters for ODrive: {} does not exist!".format(self.name))
            exit()
        
        self.params = rospy.get_param(self.name)


    def create_axis(self,axis_name,params):
        name = "position" if params["position_control"] else "velocity"
        
        # create calibration service callback
        def calibrate_srv(req=None):
            self.axis_dict[axis_name]["axis"].requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            rospy.loginfo("Calibrating {}".format(axis_name))
            then = rospy.Time.now()
            while not (
                self.axis_dict[axis_name]["axis"].motor.is_calibrated 
                and self.axis_dict[axis_name]["axis"].encoder.is_ready 
                and self.axis_dict[axis_name]["axis"].current_state == AXIS_STATE_IDLE
                ):
                if (rospy.Time.now()-then) > rospy.Duration(nsecs=1000000000):
                    then = rospy.Time.now()
                                
            # set flag to calibrated
            self.axis_dict[axis_name]["axis"].controller.pos_setpoint = self.axis_dict[axis_name]["axis"].encoder.pos_estimate
            self.axis_dict[axis_name]["axis"].controller.vel_setpoint = 0
            self.axis_dict[axis_name]["axis"].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.axis_dict[axis_name]["axis"].controller.config.control_mode = CTRL_MODE_POSITION_CONTROL if name == "position" else CTRL_MODE_VELOCITY_CONTROL
            rospy.loginfo("Control Gains: {}, {}, {}".format(
                self.axis_dict[axis_name]["axis"].controller.config.pos_gain,
                self.axis_dict[axis_name]["axis"].controller.config.vel_gain,
                self.axis_dict[axis_name]["axis"].controller.config.vel_integrator_gain
                ))
            return TriggerResponse(True,"")
        
        self.axis_dict[axis_name]["calibrate"] = rospy.Service("{}/calibrate".format(axis_name),Trigger,calibrate_srv)
        
        # create action server if no twist
        if not self.params["twist"]["enable"]:
            self.axis_dict[axis_name]["action_server"] = self.Axis_Action_Server(
                axis=self.axis_dict[axis_name]["axis"],
                name=axis_name,
                ctrl_type=name,
                cpr=self.axis_dict[axis_name]["cpr"]
            )
        # create feedback topics
        self.axis_dict[axis_name]["publisher"] = rospy.Publisher("{}/{}".format(axis_name,name),odrive_ros.msg.Float64Stamped, queue_size=1)
        self.axis_dict[axis_name]["amperage"] = rospy.Publisher("{}/{}".format(axis_name,"amperage"),odrive_ros.msg.Float64Stamped, queue_size=1)
        
        # create update routines
        def update():
            """
            Publish updates to topics.
            """
            #rospy.logdebug("Updating {} for {}".format(name,axis_name))
            msg = odrive_ros.msg.Float64Stamped(header=Header(),data=0.0)
            msg.header.stamp = rospy.Time.now()
            msg.data = self.axis_dict[axis_name]["axis"].encoder.pos_estimate if name=="position" else self.axis_dict[axis_name]["axis"].encoder.vel_estimate
            self.axis_dict[axis_name]["publisher"].publish(msg)
            
            #rospy.logdebug("Updating amperage for {}".format(axis_name))
            msg = odrive_ros.msg.Float64Stamped(header=msg.header,data=0.0)
            msg.data = self.axis_dict[axis_name]["axis"].motor.current_control.Iq_measured
            self.axis_dict[axis_name]["amperage"].publish(msg)
            
        
        self.axis_dict[axis_name]["update"] = update
                
    def connect(self):
        """
        Connect to the ODrive.
        """
        #TODO: Search for specific ODrive
        rospy.loginfo("Waiting for ODrive")
        self.ODrive = odrive.find_any() if not self.params["serial"] else odrive.find_any(serial_number=self.params["serial"])
        rospy.loginfo("Found ODrive: {}".format(hex(self.ODrive.serial_number).upper()[2:]))
        self.axis_dict = {
            self.params["axis0"]["name"] : {"axis":self.ODrive.axis0,"params":self.params["axis0"]},
            self.params["axis1"]["name"] : {"axis":self.ODrive.axis1,"params":self.params["axis1"]}
        }
        return

    def twist_cb(self,msg):
        """
        Process twist messages.
        self.twist_spacing handles diff axis spacing
        """
        rospy.logdebug("Received Twist message")
        # TODO implement timeout
        self.last_msg_time = rospy.Time.now()
        lin = msg.linear.x
        ang = msg.angular.z
        # units are m/s
        left =  (linear_velocity - angular_velocity * self.params["twist"]["axis_spacing"] / 2.0)
        right = (linear_velocity + angular_velocity * self.params["twist"]["axis_spacing"] / 2.0)
        # convert m/s to cpr/s and send to ODrive
        if self.params["twist"]["left"]:
            self.ODrive.axis0.controller.vel_setpoint = left * self.params["axis0"]["cpr"]
            self.ODrive.axis1.controller.vel_setpoint = right * self.params["axis1"]["cpr"]
        else:
            self.ODrive.axis0.controller.vel_setpoint = right * self.params["axis0"]["cpr"]
            self.ODrive.axis1.controller.vel_setpoint = left * self.params["axis1"]["cpr"]
        return

    def update(self):
        """
        Update loop for published status messages.
        """
        for axis_name in self.axis_dict.keys():
            self.axis_dict[axis_name]["update"]()