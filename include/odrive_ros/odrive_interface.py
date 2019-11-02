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
            def control_action(goal):
                """
                Action to set position of axis.
                """
                rospy.logdebug("Setting {} setpoint for {} to {}".format(name,axis_name, goal.setpoint))
                if name == "position":
                    self.axis_dict[axis_name]["axis"].controller.pos_setpoint = goal.setpoint * self.axis_dict[axis_name]["params"]["cpr"]
                    sensor_val = self.axis_dict[axis_name]["axis"].encoder.pos_estimate
                    rospy.logdebug("{}".format(self.axis_dict[axis_name]["axis"].controller.pos_setpoint))
                elif name == "velocity":
                    self.axis_dict[axis_name]["axis"].controller.vel_setpoint = goal.setpoint * self.axis_dict[axis_name]["params"]["cpr"]
                    sensor_val = self.axis_dict[axis_name]["axis"].encoder.vel_estimate
                    rospy.logdebug("{}".format(self.axis_dict[axis_name]["axis"].controller.vel_setpoint))

                
                
                then = rospy.Time.now()
                # TODO: Parameterize threshold margin
                while abs(goal.setpoint* self.axis_dict[axis_name]["params"]["cpr"]-sensor_val) > self.axis_dict[axis_name]["params"]["tolerance"]* self.axis_dict[axis_name]["params"]["cpr"]:
                    if (rospy.Time.now()-then) > rospy.Duration(nsecs=1000000000):
                        then = rospy.Time.now()
                        # publish feedback
                        self.axis_dict[axis_name]["control"].publish_feedback(
                            odrive_ros.msg.SetpointFeedback(
                                current_pos=self.axis_dict[axis_name]["axis"].encoder.pos_estimate,
                                current_vel=self.axis_dict[axis_name]["axis"].encoder.vel_estimate
                                )
                            )
                        if name == "position":
                            sensor_val = self.axis_dict[axis_name]["axis"].encoder.pos_estimate
                        elif name == "velocity":
                            sensor_val = self.axis_dict[axis_name]["axis"].encoder.vel_estimate
                            rospy.logdebug("VelSet {}".format(self.axis_dict[axis_name]["axis"].controller.vel_setpoint))
                            rospy.logdebug("VelVal {}".format(sensor_val))
                # if success
                if name == "position":
                    sensor_val = self.axis_dict[axis_name]["axis"].encoder.pos_estimate
                elif name == "velocity":
                    sensor_val = self.axis_dict[axis_name]["axis"].encoder.vel_estimate
                self.axis_dict[axis_name]["control"].set_succeeded(
                    odrive_ros.msg.SetpointResult(
                        sensor_val
                        )
                    )
                return
            
            self.axis_dict[axis_name]["control"] = actionlib.SimpleActionServer(
                "{}/{}".format(axis_name,name),
                odrive_ros.msg.SetpointAction,
                execute_cb=control_action,
                auto_start=False
                )
            self.axis_dict[axis_name]["control"].start()
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