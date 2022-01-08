#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from Robot_State import Bot_State
from WaypointManager import WaypointMananger
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
    
    def move(linear,angular):
  
    	velocity_msg.linear.x = linear
    	velocity_msg.angular.z = angular
    	self.pub.publish(velocity_msg)
        		
    def fix_error(self,P,linear_error=0,theta_error=0):
    	if linear!=0:
    	#moving in straight line
          move(P*linear_error,0)
    	else:
    	#fixing the yaw
      	  move(0.1 * np.abs(theta_error), P * -theta_error)


    def goto(self, dest_x, dest_y):
    
              # when goal is not reached
              
        while self.state != Bot_State.Goal_Reached.value: 
        
              # mathematical formula to calculate angle to be rotated
              
            theta_goal = np.arctan((dest_y - self.odom.get_position()["y"]) / dest_x - self.odom.get_position()["x"])  
            bot_theta = self.odom.get_orientation()["z"]
            theta_error = round(bot_theta - theta_goal, 2)
            rospy.loginfo("STATE:" + str(self.state))
            rospy.loginfo("THETA ERROR:" + str(theta_error))
            
             # if statement to fix yaw of bot 
            
            if self.state == Bot_State.Fixing_Yaw.value: 
            
             
             # to check whether generated errror is in within limit or not  
                 
                if np.abs(theta_error) > self.theta_precision:
                    rospy.loginfo("FIXING YAW ")
                    
             # if not then call function to correct it  
                    fix_yaw(theta_error, self.P)             
                else:
                    rospy.loginfo("YAW FIXED ! Moving toward Goal")
                    self.state = Bot_State.Moving_Straight.value
            
             #  the bot is correctly oriented so it move towards goal 
             
            elif self.state == Bot_State.Moving_Straight.value:
            
            # mathematical formula to calculate positional error
            
                position_error = np.sqrt(pow(dest_y - self.odom.get_position()["y"], 2) + pow(dest_x - self.odom.get_position()["x"], 2)) 
                rospy.loginfo("POSITION ERROR: " + str(position_error))
                
                if position_error > self.dist_precision and np.abs(theta_error) < self.theta_precision: 
                    rospy.loginfo("Moving Straight")
                    move_straight(position_error, self.P )
                    
                elif np.abs(theta_error) > self.theta_precision: 
                    rospy.loginfo("Going out of line!")
                    self.state = Bot_State.Fixing_Yaw.value
                    
                    #  goal reached 
                elif position_error < self.dist_precision:  
                    rospy.loginfo("GOAL REACHED")
                    self.state = Bot_State.Goal_Reached.value

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

        rospy.loginfo("Bot has arrived at destination")


