import yaml
import rospy
from geometry_msgs.msg import Twist

class CurveTracer:

    def __init__(self):
        rospy.init_node("CurveTracer", anonymous=True)
        waypoint = WaypointManager(100)
        odom = OdomSubscriber()

        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        velocity_msg = Twist()
        # with open("data.yaml", 'r') as stream:
        #   data_loaded = yaml.safe_load(stream)
        # self.theta_precision = data_loaded["curve_tracer_controller"]["theta_precision"]
        # self.dist_precision = data_loaded["curve_tracer_controller"]["distance_precision"]
        # self.P = data_loaded["curve_tracer_controller"]["pid"]["p"]


        self.theta_precision = rospy.get_param("theta_precision")
        self.dist_precision =  rospy.get_param("distance_precision")
        self.P = rospy.get_param("pid")



