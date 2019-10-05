import rospy
import actionlib
from geometry_msgs.msg import Twist

from std_srvs.srv import Trigger,TriggerResponse
import odrive_ros.msg
from odrive_ros.srv import Float64, Float64Response

#import odrive_test_funcs as odrive_funcs

class ODrive_ROS:

    def __init__(self,rosparam_name):
        self.name = rosparam_name
        # load parameters
        self.load_params()
        # connect
        self.connect()

        #self.start_topics()

        self.create_axis(self.params["axis0"]["name"],self.params["axis0"])
        self.create_axis(self.params["axis1"]["name"],self.params["axis1"])

    def load_params(self):
        """
        Loads ROS parameters using parameter name.
        """
        if not rospy.has_param(self.name):
            rospy.logerr("Parameters for ODrive: {} does not exist!".format(self.name))
            exit()
        
        self.params = rospy.get_param(self.name)

    def create_axis(self,axis_name,params):
        """
        Assembles axis's components based on params
        """
        # create calibrate service or just run calibration
        cb = self.calibrate_srv_factory(axis_name,params["index_search"])
        if params["calibrate_on_startup"]:
            cb()
        self.axis_dict[axis_name]["calibrate"] = rospy.Service("{}/calibrate".format(axis_name),Trigger,cb)
        
        # create control service (pos/vel/none)
        cb = self.control_srv_factory(axis_name,params["enable_position_control"],params["enable_velocity_control"])
        name = "position" if params["enable_position_control"] else "velocity" if params["enable_velocity_control"] else "ERROR"
        self.axis_dict[axis_name]["control"] = rospy.Service("{}/{}".format(axis_name,name),Float64,cb)

        # create control action server
        cb = self.control_action_factory(axis_name,params["enable_position_control"],params["enable_velocity_control"])
        self.axis_dict[axis_name]["control"] = actionlib.SimpleActionServer(
            "{}/{}".format(axis_name,name),
            odrive_ros.msg.SetpointAction,
            execute_cb=cb,
            auto_start=False
            )
        self.axis_dict[axis_name]["control"].start()

    def control_action_factory(self,axis_name,pos=True,vel=False):
        """
        Creates a control action callback for the axis.
        Will be position or velocity based on parameter.
        """
        if pos and vel:
            rospy.logerr("Cannot use both velocity and control on axis: {}! Defaulting to position only".format(axis_name))
        elif pos:
            def control_action(goal):
                """
                Action to set position of axis.
                """
                #self.axis_dict[axis_name]["axis"].controller.pos_setpoint = req.setpoint
                rospy.loginfo("Setting position setpoint for {} to {}".format(axis_name, goal.setpoint))
                for i in range(5):
                    # publish feedback
                    self.axis_dict[axis_name]["control"].publish_feedback(odrive_ros.msg.SetpointFeedback(i)) 
                # if success
                self.axis_dict[axis_name]["control"].set_succeeded(odrive_ros.msg.SetpointResult(0.0))
                return
        elif vel:
            def control_action(goal):
                """
                Service to set velocity of axis.
                """
                #self.axis_dict[axis_name]["axis"].controller.vel_setpoint = req.setpoint
                rospy.loginfo("Setting velocity setpoint for {} to {}".format(axis_name, goal.setpoint))
                for i in range(5):
                    # publish feedback
                    self.axis_dict[axis_name]["control"].publish_feedback(odrive_ros.msg.SetpointFeedback(i))              
                # if success
                self.axis_dict[axis_name]["control"].set_succeeded(odrive_ros.msg.SetpointResult(0.0))
                return
        return control_action

    def calibrate_srv_factory(self,axis_name,index_search=False):
        """
        Creates a calibrate srv callback for the axis.
        """
        def calibrate_srv(req=None):
            #self.axis_dict[axis_name]["axis"].requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH if index_search else AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            rospy.loginfo("Calibrating {}".format(axis_name))
            # wait until calibrated
            # set flag to calibrated
            return TriggerResponse(True,"")
        return calibrate_srv

    def control_srv_factory(self,axis_name,pos=True,vel=False):
        """
        Creates a control srv callback for the axis.
        Will be position or velocity based on parameter.
        """
        if pos and vel:
            rospy.logerr("Cannot use both velocity and control on axis: {}! Defaulting to position only".format(axis_name))
        elif pos:
            def control_srv(req=None):
                """
                Service to set position of axis.
                """
                #self.axis_dict[axis_name]["axis"].controller.pos_setpoint = req.setpoint
                rospy.loginfo("Setting position setpoint for {} to {}".format(axis_name, req.setpoint))
                return Float64Response(0.0,"")
        elif vel:
            def control_srv(req):
                """
                Service to set velocity of axis.
                """
                #self.axis_dict[axis_name]["axis"].controller.vel_setpoint = req.setpoint
                rospy.loginfo("Setting velocity setpoint for {} to {}".format(axis_name, req.setpoint))
                return Float64Response(0.0,"")
        return control_srv
                


    def connect(self):
        """
        Connect to the ODrive.
        """
        class empty(object):
            pass
        self.ODrive = empty()
        self.ODrive.axis0 = empty()
        self.ODrive.axis1 = empty()

        self.axis_dict = {
            self.params["axis0"]["name"] : {"axis":self.ODrive.axis0},
            self.params["axis1"]["name"] : {"axis":self.ODrive.axis1}
        }


        return

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