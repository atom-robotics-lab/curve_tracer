#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from Robot_State import Bot_State
from WaypointManager import WaypointManager
from OdomSubscriber import OdomSubscriber
import numpy as np


class CurveTracer:

    def __init__(self):

        rospy.init_node("CurveTracer", anonymous=True)
        # objects for waypoint and odom
        self.waypoint = WaypointManager(100)
        self.odom = OdomSubscriber()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.velocity_msg = Twist()
        # making the parameters as memebers of the class
        self.theta_precision = rospy.get_param("curve_tracer_controller/theta_precision")
        self.dist_precision = rospy.get_param("curve_tracer_controller/distance_precision")
        self.P = rospy.get_param("curve_tracer_controller/pid/p")
        self.D = rospy.get_param("curve_tracer_controller/pid/d")
        self.I = rospy.get_param("curve_tracer_controller/pid/i")
        self.state = Bot_State.IdleState.value
        
                            # ------------------ CONTROL LOOP ------------------ #
    def control_loop(self):
        rate = rospy.Rate(10)
        self.move(0, 0)  # stop the bot initially if it is already in motion
        rospy.loginfo("Waypoints : " + str(self.waypoint.way_point()))
         
        while not rospy.is_shutdown() :
             if self.waypoint.get_next_waypoint == None:
              self.move(0,0)
              rospy.loginfo("GOAL REACHED")
             else:
              (x, y) = self.waypoint.get_next_waypoint()
              rospy.loginfo("Moving to point: ({},{}) ".format(x, y))
              self.goto(round(x, 2), round(y, 2))
              self.move(0, 0)
              rate.sleep()

                          # ------------------ GOTO FUNCTION ------------------ #
    def goto(self, dest_x, dest_y):
        self.state = Bot_State.Fixing_Yaw.value # initialling bot state to start with yaw fixing

        rospy.loginfo(self.state)

        theta_goal = np.arctan((dest_y - self.odom.get_position()["y"]) / dest_x - self.odom.get_position()["x"])
        bot_theta = self.odom.get_orientation("euler")["yaw"]
        theta_error = round(bot_theta - theta_goal, 2)
        
        rospy.loginfo(theta_goal)
        rospy.loginfo(bot_theta)
        rospy.loginfo(theta_error)

        # when particular point goal is not reached
        while self.state != Bot_State.Goal_Reached.value:
            # mathematical formula to calculate angle to be rotated
            theta_goal = np.arctan((dest_y - self.odom.get_position()["y"]) / dest_x - self.odom.get_position()["x"])
            bot_theta = self.odom.get_orientation("euler")["yaw"]
            theta_error = round(bot_theta - theta_goal, 2)
            

            # if statement to fix yaw of bot

            if self.state == Bot_State.Fixing_Yaw.value:

                # to check whether generated errror is in within limit or not
                
              while(np.abs(theta_error) > self.theta_precision):
            
                #printing of essential data
                rospy.loginfo("Bot Current THETA: "+ str(bot_theta))
                rospy.loginfo("STATE:" + str(self.state))
                rospy.loginfo("THETA ERROR:" + str(theta_error))
                rospy.loginfo("FIXING YAW ")

                # mathematical formula to calculate angle to be rotated
                theta_goal = np.arctan((dest_y - self.odom.get_position()["y"]) / dest_x - self.odom.get_position()["x"])
                bot_theta = self.odom.get_orientation("euler")["yaw"]
                theta_error = round(bot_theta - theta_goal, 2)

                # if not then call function to correct it
                self.fix_error(self.P,0,theta_error)

              rospy.loginfo("YAW FIXED ! Moving toward Goal")
              self.state = Bot_State.Moving_Straight.value

            #  the bot is correctly oriented so it move towards goal

            if self.state == Bot_State.Moving_Straight.value:

                # mathematical formula to calculate positional error

                position_error = np.sqrt(pow(dest_y - self.odom.get_position()["y"], 2) + pow(dest_x - self.odom.get_position()["x"], 2))
                rospy.loginfo("POSITION ERROR: " + str(position_error))

                if position_error > self.dist_precision and np.abs(theta_error) < self.theta_precision:
                    rospy.loginfo("Moving Straight")
                    self.fix_error(self.P,position_error,0)

                elif np.abs(theta_error) > self.theta_precision:
                    rospy.loginfo("Going out of line!")
                    self.state = Bot_State.Fixing_Yaw.value

                    #  goal reached 
                elif position_error < self.dist_precision:
                    rospy.loginfo("GOAL REACHED")
                    self.state = Bot_State.Goal_Reached.value

                   # ------------------ MOVE ------------------ #
    def move(self, linear, angular):

        self.velocity_msg.linear.x = linear
        self.velocity_msg.angular.z = angular
        self.pub.publish(self.velocity_msg)
        
                  # ------------------ FIX ERROR ------------------ #

    def fix_error(self, P, linear_error, orien_error):
        if linear_error != 0:
            # moving in straight line
            self.move(P * linear_error, 0)
        if orien_error != 0:
            # fixing the yaw
            # using negative P because bot rotates clockwise with -ve angular velocity in gazebo
            rospy.loginfo("Fixing with velocity: ")
            rospy.loginfo(-P*orien_error)
            self.move(0, -P*orien_error)
            
            
   

              # ------------------CLASS CALLING ------------------ #
curve = CurveTracer()
curve.goto(1, 0)
rospy.spin()
