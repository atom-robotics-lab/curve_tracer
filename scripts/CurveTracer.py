#! /usr/bin/env python3

import yaml
import rospy
from geometry_msgs.msg import Twist
from enums import Bot_State
from WaypointManager import WaypointMananger
from OdomSubscriber import OdomSubscriber


class CurveTracer:

    def __init__(self):

        rospy.init_node("CurveTracer", anonymous=True)

        # objects for waypoint and odom
        self.waypoint = WaypointManager(100)
        self.odom = OdomSubscriber()

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.velocity_msg = Twist()
        # with open("data.yaml", 'r') as stream:
        #   data_loaded = yaml.safe_load(stream)
        # self.theta_precision = data_loaded["curve_tracer_controller"]["theta_precision"]
        # self.dist_precision = data_loaded["curve_tracer_controller"]["distance_precision"]
        # self.P = data_loaded["curve_tracer_controller"]["pid"]["p"]

        # making the parameters as memebers of the class
        self.theta_precision = rospy.get_param("theta_precision")
        self.dist_precision = rospy.get_param("distance_precision")
        self.P = rospy.get_param("pid")
        self.state = 0


    def goto(self, dest_x, dest_y):
        while self.state != Bot_State.Goal_Reached.value:
            theta_goal = np.arctan((dest_y - self.odom.get_position()["y"]) / dest_x - self.odom.get_position()["x"])
            bot_theta = self.odom.get_position()["z"]
            theta_error = round(bot_theta - theta_goal, 2)
            rospy.loginfo("STATE:" + str(self.state))
            rospy.loginfo("THETA ERROR:" + str(theta_error))
            if self.state == Bot_State.Fixing_Yaw.value:
                if np.abs(theta_error) > self.theta_precision:
                    rospy.loginfo("FIXING YAW ")
                    fix_yaw(theta_error, 1.7)
                else:
                    rospy.loginfo("YAW FIXED ! Moving toward Goal")
                    self.state = Bot_State.Moving_Straight.value

            elif self.state == Bot_State.Moving_Straight.value:
                position_error = np.sqrt(pow(dest_y - self.odom.get_position()["y"], 2) + pow(dest_x - self.odom.get_position()["x"], 2))
                rospy.loginfo("POSITION ERROR: " + str(position_error))
                if position_error > self.dist_precision and np.abs(theta_error) < self.theta_precision:
                    rospy.loginfo("Moving Straight")
                    move_straight(position_error, self.P )
                elif np.abs(theta_error) > self.theta_precision:
                    rospy.loginfo("Going out of line!")
                    self.state = 0
                elif position_error < self.dist_precision:
                    rospy.loginfo("GOAL REACHED")
                    self.state = 2

    def control_loop(self):
        rate = rospy.Rate(10)

        move(0, 0)  # stop the bot initially if it is already in motion

        rospy.loginfo("Waypoints : ", self.waypoint.way_point())

        while not rospy.is_shutdown() and self.waypoint.get_next_waypoint != None:
            (x, y) = self.waypoint.get_next_waypoint()

            rospy.loginfo("Moving to point: ({},{}) ".format(x, y))

            goto(round(x, 2), round(y, 2))

            rate.sleep()

        move(0, 0)  # stop the bot

        self.state = 2  # state -2 -> goal reached

        rospy.loginfo("Bot has arrived at destination")



