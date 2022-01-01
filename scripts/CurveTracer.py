#! /usr/bin/env python3

import yaml
import rospy
from geometry_msgs.msg import Twist
from WaypointManager import WaypointMananger

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

        self.waypoint=waypoint
        self.theta_precision = rospy.get_param("theta_precision")
        self.dist_precision =  rospy.get_param("distance_precision")
        self.P = rospy.get_param("pid")
        self.state=0

    def control_loop():
        rate = rospy.Rate(10)

        move(0,0) #stop the bot initially if it is already in motion

        rospy.loginfo("Waypoints : ",self.waypoint.way_point())

        while not rospy.is_shutdown() and waypoints.get_next_waypoint!=None:

            (x,y) = waypoints.get_next_waypoint()                      
                                                       
            rospy.loginfo("Moving to point: ({},{}) ".format(x,y))

            goto(round(x, 2), round(y, 2))

            rate.sleep()
        
        move(0,0)           #stop the bot
        self.state=2        #state -2 -> goal reached

        rospy.loginfo("Bot has arrived at destination")

        

            



        
        








