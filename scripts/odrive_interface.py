import rospy
from geometry_msgs.msg import Twist

from std_srvs.srv import Trigger,TriggerResponse

from odrive_ros.srv import Float64, Float64Response

#import odrive_test_funcs as odrive_funcs

class ODrive_Interface:

    def __init__(self,rosparam_name):
        self.name = rosparam_name
        self.calibrated0 = False
        self.calibrated1 = False
        # load parameters
        self.load_params()
        # connect
        self.connect()
        # calibrate
        if self.calibrate_on_startup0:
            self.calibrate0_srv()
        if self.calibrate_on_startup1:
            self.calibrate1_srv()

        self.start_topics()
        self.start_services()

    def load_params(self):
        """
        Loads ROS parameters using parameter name.
        """
        if not rospy.has_param(self.name):
            rospy.logerr("Parameters for ODrive: {} does not exist!".format(self.name))
            exit()
        
        params = rospy.get_param(self.name)
        # Id and serial values
        self.serial_id = params["serial"]
        #self.serial_port = params["port"]

        # Twist topic parameters
        self.twist_en = params["twist"]["enable"]
        self.twist_spacing = params["twist"]["axis_spacing"]

        # Axis 0
        axis = params["axis0"]
        self.name0 = axis["name"]
        self.en_pos_serv0 = axis["enable_position_service"]
        self.en_vel_serv0 = axis["enable_velocity_service"]
        self.cpr_gain0 = axis["cpr_conversion_factor"]
        self.calibrate_on_startup0 = axis["calibrate_on_startup"]
        self.index_search0 = axis["index_search"]
        # Axis 1
        axis = params["axis1"]
        self.name1 = axis["name"]
        self.en_pos_serv1 = axis["enable_position_service"]
        self.en_vel_serv1 = axis["enable_velocity_service"]
        self.cpr_gain1 = axis["cpr_conversion_factor"]
        self.calibrate_on_startup1 = axis["calibrate_on_startup"]
        self.index_search1 = axis["index_search"]

    def connect(self):
        """
        Connect to the ODrive.
        """
        self.ODrive = None
        return

    def calibrate0_srv(self,req=None):
        """
        Calibrate axis0.
        """
        rospy.loginfo("Calibrating {}".format(self.name0))
        return TriggerResponse(True,"")

    def calibrate1_srv(self,req=None):
        """
        Calibrate axis1.
        """
        rospy.loginfo("Calibrating {}".format(self.name1))
        return TriggerResponse(True,"")

    def pos0_srv(self,req):
        """
        Service to set position of axis 0.
        """
        rospy.loginfo("Setting position setpoint for {} to {}".format(self.name0, req.setpoint))
        return Float64Response(0.0,"")

    def pos1_srv(self,req):
        """
        Service to set position of axis 1.
        """
        rospy.loginfo("Setting position setpoint for {} to {}".format(self.name1, req.setpoint))
        return Float64Response(0.0,"")

    def vel0_srv(self,req):
        """
        Service to set velocity of axis 0.
        """
        rospy.loginfo("Setting velocity setpoint for {} to {}".format(self.name0, req.setpoint))
        return Float64Response(0.0,"")

    def vel1_srv(self,req):
        """
        Service to set velocity of axis 1.
        """
        rospy.loginfo("Setting velocity setpoint for {} to {}".format(self.name1, req.setpoint))
        return Float64Response(0.0,"")

    def start_topics(self):
        if self.twist_en:
            self.sub = rospy.Publisher(self.name, Twist, self.twist_cb)
        return

    def twist_cb(self,msg):
        """
        Process twist messages.
        self.twist_spacing handles diff axis spacing
        """
        rospy.loginfo("Received Twist message")
        return

    def start_services(self):
        """
        Start up services for both axes.
        """
        if not self.calibrated0:
            self.calibrate_srv0 = rospy.Service("{}_calibrate".format(self.name0),Trigger,self.calibrate0_srv)
        if not self.calibrated1:
            self.calibrate_srv1 = rospy.Service("{}_calibrate".format(self.name1),Trigger,self.calibrate1_srv)

        # Position
        if self.en_pos_serv0:
            self.pos0 = rospy.Service("{}_set_position".format(self.name0),Float64,self.pos0_srv)
        if self.en_pos_serv1:
            self.pos1 = rospy.Service("{}_set_position".format(self.name1),Float64,self.pos1_srv)

        # Velocity
        if self.en_vel_serv0:
            self.vel0 = rospy.Service("{}_set_velocity".format(self.name0),Float64,self.vel0_srv)
        if self.en_vel_serv1:
            self.vel1 = rospy.Service("{}_set_velocity".format(self.name1),Float64,self.vel1_srv)