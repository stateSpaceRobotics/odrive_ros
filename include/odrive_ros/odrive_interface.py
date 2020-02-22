import math
import time

import odrive
from odrive.enums import *

import rospy
import actionlib
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger,TriggerResponse
from std_msgs.msg import Header
import odrive_ros
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
                "{}/{}".format(self._name,self._type), # TODO Fix
                odrive_ros.msg.SetpointAction,
                execute_cb=cb,
                auto_start=False
                )
            self._as.start()

        def pos_cb(self, goal):
            self._axis.controller.pos_setpoint = goal.setpoint * self._cpr
            rospy.logdebug("Received position goal on {}: {}".format(self._name,self._axis.controller.pos_setpoint))
            rate = rospy.Rate(2)

            # supply feedback until to setpoint
            while abs(goal.setpoint-self._axis.encoder.pos_estimate/self._cpr) > goal.tolerance and not rospy.is_shutdown():
                # preempt check; if preempted exit
                if self._as.is_preempt_requested():
                    rospy.loginfo("{}/{} has been preempted!".format(self._name,self._type))
                    # motor should stop where it is
                    self._axis.controller.pos_setpoint = self._axis.encoder.pos_estimate
                    self._as.set_preempted(
                        result=odrive_ros.msg.SetpointResult(
                            final=self._axis.encoder.pos_estimate/self._cpr
                        )
                    )
                    return

                self._as.publish_feedback(
                    odrive_ros.msg.SetpointFeedback(
                        current_pos=self._axis.encoder.pos_estimate/self._cpr,
                        current_vel=self._axis.encoder.vel_estimate/self._cpr
                    )
                )
                rate.sleep()
            # success hit
            self._as.set_succeeded(
                odrive_ros.msg.SetpointResult(
                    final=self._axis.encoder.pos_estimate/self._cpr
                )
            )
            return

        def vel_cb(self, goal):
            self._axis.controller.vel_setpoint = goal.setpoint * self._cpr
            rospy.logdebug("Received velocity goal on {}: {}".format(self._name,self._axis.controller.vel_setpoint))
            rate = rospy.Rate(2)
            rospy.logdebug("goal: {}".format(goal.setpoint * self._cpr))
            # supply feedback until to setpoint
            while abs(goal.setpoint-self._axis.encoder.vel_estimate/self._cpr) > goal.tolerance and not rospy.is_shutdown():
                rospy.logdebug(self._axis.encoder.vel_estimate)
                # preempt check; if preempted exit
                if self._as.is_preempt_requested():
                    rospy.loginfo("{}/{} has been preempted!".format(self._name,self._type))
                    # motor should stop where it is
                    self._axis.controller.vel_setpoint = self._axis.encoder.vel_estimate
                    self._as.set_preempted(
                        result=odrive_ros.msg.SetpointResult(
                            final=self._axis.encoder.vel_estimate/self._cpr
                        )
                    )
                    return

                self._as.publish_feedback(
                    odrive_ros.msg.SetpointFeedback(
                        current_pos=self._axis.encoder.pos_estimate/self._cpr,
                        current_vel=self._axis.encoder.vel_estimate/self._cpr
                    )
                )
                rate.sleep()
            # success hit
            self._as.set_succeeded(
                odrive_ros.msg.SetpointResult(
                    final=self._axis.encoder.vel_estimate/self._cpr
                )
            )
            return

    def __init__(self,rosparam_name):
        self.name = rosparam_name
        if not rospy.has_param(self.name):
            rospy.logerr("ODrive sub-node ({}) has no parameters!".format(self.name))
        else:
            # connect
            self.connect()
            # load parameters
            self.load_params()
            # finih parameters
            self.axis_dict = {
                self.params["axis0"]["name"] : {"axis":self.ODrive.axis0,"params":self.params["axis0"]},
                self.params["axis1"]["name"] : {"axis":self.ODrive.axis1,"params":self.params["axis1"]}
            }

            self.create_axis(self.params["axis0"])
            self.create_axis(self.params["axis1"])
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
        self.params["axis0"]["axis"] = self.ODrive.axis0
        self.params["axis1"]["axis"] = self.ODrive.axis1


    def create_axis(self,axis_params):
        name = axis_params["name"]
        ctrl_type = axis_params["control_mode"]
        if ctrl_type not in ["position", "velocity"]:
            rospy.logerr("Invalid control_mode parameter for {}!".format(name))
            raise ValueError("ctrl_type must be \"position\"/\"velocity\"")
        
        # create calibration service callback
        def calibrate_srv(req=None):
            if not axis_params["axis"].motor.is_calibrated or not axis_params["axis"].encoder.is_ready:
                axis_params["axis"].requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                rospy.loginfo("Calibrating {}".format(name))
                then = rospy.Time.now()
                while not (
                    self.axis_dict[name]["axis"].motor.is_calibrated 
                    and self.axis_dict[name]["axis"].encoder.is_ready 
                    and self.axis_dict[name]["axis"].current_state == AXIS_STATE_IDLE
                    ):
                    if (rospy.Time.now()-then) > rospy.Duration(nsecs=1000000000):
                        then = rospy.Time.now()
                                
            # set flag to calibrated
            axis_params["axis"].controller.pos_setpoint = axis_params["axis"].encoder.pos_estimate
            axis_params["axis"].controller.vel_setpoint = 0
            axis_params["axis"].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            axis_params["axis"].controller.config.vel_limit = axis_params["cpr"]*100.0
            axis_params["axis"].controller.config.control_mode = CTRL_MODE_POSITION_CONTROL if ctrl_type == "position" else CTRL_MODE_VELOCITY_CONTROL
            rospy.logdebug("Control Gains: {}, {}, {}".format(
                axis_params["axis"].controller.config.pos_gain,
                axis_params["axis"].controller.config.vel_gain,
                axis_params["axis"].controller.config.vel_integrator_gain
                ))
            return TriggerResponse(True,"")
        if axis_params["calibrate_on_startup"]:
            calibrate_srv()
            
        axis_params["calibrate"] = rospy.Service("{}/calibrate".format(name),Trigger,calibrate_srv)
        
        # create action server if no twist
        if not self.params["twist"]["enable"]:
            axis_params["action_server"] = self.Axis_Action_Server(
                axis=axis_params["axis"],
                name=name,
                ctrl_type=ctrl_type,
                cpr=axis_params["cpr"]
            )
        # create feedback topics
        axis_params["publisher"] = rospy.Publisher("{}/{}".format(name,ctrl_type),odrive_ros.msg.Float64Stamped, queue_size=1)
        axis_params["amperage"] = rospy.Publisher("{}/{}".format(name,"amperage"),odrive_ros.msg.Float64Stamped, queue_size=1)
        
        # create update routines
        def update():
            """
            Publish updates to topics.
            """
            #rospy.logdebug("Updating {} for {}".format(name,axis_name))
            msg = odrive_ros.msg.Float64Stamped(header=Header(),data=0.0)
            msg.header.stamp = rospy.Time.now()
            if ctrl_type == "position":
                msg.data = axis_params["axis"].encoder.pos_estimate
            elif ctrl_type == "velocity":
                msg.data = axis_params["axis"].encoder.vel_estimate
            axis_params["publisher"].publish(msg)
            
            #rospy.logdebug("Updating amperage for {}".format(axis_name))
            msg = odrive_ros.msg.Float64Stamped(header=msg.header,data=0.0)
            msg.data = axis_params["axis"].motor.current_control.Iq_measured
            axis_params["amperage"].publish(msg)
        axis_params["update"] = update
                
    def connect(self):
        """
        Connect to the ODrive.
        """
        #TODO: Search for specific ODrive
        if rospy.has_param("{}/serial".format(self.name)):
            serial = rospy.get_param("{}/serial".format(self.name))
            rospy.loginfo("Waiting for ODrive (serial: {})".format(serial))            
            self.ODrive = odrive.find_any(serial_number=serial)
        else:
            rospy.loginfo("Waiting for any ODrive")
            self.ODrive = odrive.find_any()
        rospy.loginfo("Found ODrive: {}".format(hex(self.ODrive.serial_number).upper()[2:]))
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
        left =  (lin - ang * self.params["twist"]["axis_spacing"] / 2.0)
        right = (lin + ang * self.params["twist"]["axis_spacing"] / 2.0)
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
        self.params["axis0"]["update"]()
        self.params["axis1"]["update"]()