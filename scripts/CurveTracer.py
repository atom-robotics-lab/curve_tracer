#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from Robot_State import Bot_State
from WaypointManager import WaypointManager
from OdomSubscriber import OdomSubscriber
import numpy as np
import time 
from curve_tracer.msg import PID_error_msg 




                                    #-------------CLASS--------------#
class CurveTracer:

    def __init__(self):

        rospy.init_node("CurveTracer", anonymous=True)
        
        self.waypoint = WaypointManager(100)
        
        self.odom = OdomSubscriber()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.error_pub = rospy.Publisher('/error_CurveTracer', PID_error_msg, queue_size=10)
        self.velocity_msg = Twist()
        self.error = PID_error_msg()
        self.error.angular = 0 
        self.error.linear = 0
        
               
        self.theta_precision = rospy.get_param("curve_tracer_controller/theta_precision")
        self.dist_precision = rospy.get_param("curve_tracer_controller/distance_precision")
        
        
        #-----      uncomment this only when you want to tune your PID         ------#
        #             `----> first start PlotGraph file then this in tuning mode     #
        #             `----> As PlotGraph file publishes Kp , Kd , Ki                #
        from curve_tracer.msg import PID_constants                                   #
        rospy.Subscriber('/PID_constants',PID_constants, self.constant)              #
        PID_constant = PID_constants()                                               #
        self.Kp = 0                                                                  #
        self.Kd = 0                                                                  #
        self.Ki = 0                                                                  #
        #----------------------------------------------------------------------------#
        
        #------          uncomment durning normal operation                   ------- #
        #self.Kp = rospy.get_param("curve_tracer_controller/pid/p")                   #
        #self.Kd = 1   # rospy.get_param("curve_tracer_controller/pid/d")             #
        #self.Ki = 1   # rospy.get_param("curve_tracer_controller/pid/i")             #
        #-----------------------------------------------------------------------------#
        
        
        self.linear_P = 0 
        self.linear_I = 0
        self.linear_D = 0
        self.linear_previous_error = 0
        self.angular_P = 0
        self.angular_I = 0
        self.angular_D = 0
        self.orien_previous_error  = 0
        
        self.state = Bot_State.IdleState.value
        self.previous_time = time.time() 
        self.current_time  = time.time()
        
    def constant(self,PID_value):
         self.Kp = PID_value.proportional 
         self.Kd = PID_value.derivative
         self.Ki = PID_value.integral

                                   #--------------CONTROL LOOP--------------#
    def control_loop(self):
        rate = rospy.Rate(10)
        self.move(0, 0) 
        rospy.loginfo("Waypoints : " + str(self.waypoint.way_point()))
         
        while not rospy.is_shutdown() :
             if self.waypoint.get_next_waypoint == None:
              self.move(0,0)
              rospy.loginfo("GOAL REACHED")
             else:
              (x, y) = self.waypoint.get_next_waypoint()
              rospy.loginfo("Moving to point: ({},{}) ".format(x, y))
              self.goto(round(50, 2), round(50, 2))
              rate.sleep()
              
         
                                 #--------------GOTO FUNCTION--------------#
    def goto(self,dest_x,dest_y):
     self.state = Bot_State.IdleState.value
    
     while self.state != Bot_State.Goal_Reached.value:
     
         ls  = self.vector(dest_x,dest_y)
         bot_theta_error = ls[2]
         bot_position_error = ls[3]
         ##print("************************")
         #print("*******BOT INFO*********")
         #print("************************")
         #rospy.loginfo("bot vector --->" + str(ls[0]))
         #rospy.loginfo("position vector --->" + str(ls[1]))
         #rospy.loginfo("theta error: ---> " + str(ls[2]))
         #rospy.loginfo("position error: ---> " + str(ls[3]))
         #rospy.loginfo("current position is :---->" + str(ls[4]) + " " + str(ls[5]) ) 
         #rospy.loginfo("going to ---> " +str(dest_x) +" "+ str(dest_y)) 
        
        
         while((np.abs(bot_theta_error) > self.theta_precision) or (np.abs(bot_position_error) > self.dist_precision)):
              
              while (np.abs(bot_theta_error) > self.theta_precision) :  
          #      print("************************")
          #      print("***** FIXING YAW *******")
           #     print("************************")
                ls  = self.vector(dest_x,dest_y)
                bot_theta_error = ls[2]
                bot_position_error = ls[3]
           #     rospy.loginfo("bot vector --->" + str(ls[0]))
           #     rospy.loginfo("position vector --->" + str(ls[1]))
          #      rospy.loginfo("theta error: ---> " + str(ls[2]))
           #     rospy.loginfo("position error: ---> " + str(ls[3]))
           #     rospy.loginfo("current position is :---->" + str(ls[4]) + " " + str(ls[5])) 
          #      rospy.loginfo("going to ---> " + str(dest_x) +" "+ str(dest_y)) 
                self.PID_Controller(0, bot_theta_error) 
                
                
                            
          #    rospy.loginfo (" YAW FIXED ! Moving straight .. " )                    
          #    rospy.loginfo("we have to reach: " + str(dest_x) + " " + str(dest_y))
              
              while (np.abs(bot_position_error) > self.dist_precision):
           #            print("************************")
           #            print("*** Moving Straight ****")
          #             print("************************")                                                               
                       ls  = self.vector(dest_x,dest_y)
                       bot_theta_error = ls[2]
                       bot_position_error = ls[3]
           #            rospy.loginfo("bot vector --->" + str(ls[0]))
           #            rospy.loginfo("position vector --->" + str(ls[1]))
           #            rospy.loginfo("theta error: ---> " + str(ls[2]))
           #            rospy.loginfo("position error: ---> " + str(ls[3]))
           #            rospy.loginfo("current position is :---->" + str(ls[4]) + " " + str(ls[5]))
           #            rospy.loginfo("going to ---> " +str(dest_x) +" "+ str(dest_y)) 
                       self.PID_Controller(bot_position_error ,0)   
                       
                       
                       while (np.abs(bot_theta_error) > self.theta_precision) : 
           #               print("*******************************")
           #               print("*FIXING YAW && MOVING STRAIGHT*")
           #               print("*******************************")                          
           #               rospy.loginfo("bot vector --->" + str(ls[0]))
           #               rospy.loginfo("position vector --->" + str(ls[1]))
           #               rospy.loginfo("theta error: ---> " + str(ls[2]))
           #               rospy.loginfo("position error: ---> " + str(ls[3]))
           #               rospy.loginfo("current position is :---->" + str(ls[4]) + " " + str(ls[5]))
           #               rospy.loginfo("going to ---> " +str(dest_x) +" "+ str(dest_y)) 
                          self.PID_Controller(0,bot_theta_error) 
                          ls  = self.vector(dest_x,dest_y)
                          bot_theta_error = ls[2]
                          bot_position_error = ls[3]

                  
                    
         if bot_position_error < self.dist_precision and bot_theta_error < self.theta_precision :
           #       print("*****************************")
           #       print("**HURRAY !! GOAL REACHED*****")
           #       print("*****************************")                      
                  self.state = Bot_State.Goal_Reached.value 
                  self.move(0,0)   
                              
              
              
              
                                    #--------------MOVE--------------#
    def move(self, linear, angular):

        self.velocity_msg.linear.x = linear
        self.velocity_msg.angular.z = angular
        self.pub.publish(self.velocity_msg)
        
        
                                    #--------------FIX ERROR--------------#

    ''' def fix_error(self, linear_error, orien_error):
        
        if linear_error != 0:
            
            # moving in straight line
            self.move(self.Kp*linear_error, 0)
            
        if orien_error != 0:           
            # fixing the yaw     
             self.move(0,self.Kp*-1*orien_error)'''
             
             
    def PID_Controller(self , linear_error , orien_error ):
    	
        '''
    	Inital setup :
    	       
    	       dt : PID controller duty cycle 
    	'''

        self.current_time = time.time() 
        dt = self.current_time - self.previous_time 
        
        
        '''
    	 Linear PID Controller :
    	 
    	 	linear_P : Proportional Controller - Kp * (linear_error) 
    	 	linear_D : Driffential  Controller - Kd * (difference b/w current & previous error) / dt  
    	 	linear_I : Integral     Controller - Ki * (linear_error) * dt + (previous Integral Error) 
    	   
    	'''  
        if linear_error != 0:
             self.linear_P =  (1) * (linear_error)
             self.linear_D = ((self.Kd) * (linear_error - self.linear_previous_error )) / dt 
             self.linear_I =  (self.Ki) * (linear_error) * dt  
             linear_Correction = self.linear_P #+ self.linear_I + self.linear_D 
             self.move(linear_Correction , 0 )
    	
   
        '''
    	 Angular PID Controller :
    	 
    	 	angular_P : Proportional Controller - Kp * (angular_error) 
    	 	angular_D : Driffential  Controller - Kd * (difference b/w current & previous error) / dt  
    	 	angular_I : Integral     Controller - Ki * (error) * dt + (previous Integral Error) 
    	   
    	'''  
        if orien_error != 0:         
             self.angular_P =  (self.Kp) *  (orien_error) 
             self.angular_D = ((self.Kd) * ((orien_error - self.orien_previous_error )) / dt )
             self.angular_I =  (self.Ki) * ((orien_error) * dt + self.angular_I ) 
             angular_Correction = self.angular_P + self.angular_D + self.angular_I
             self.error.angular = round(angular_Correction ,2)
             print("KP:", self.angular_P,self.Kp )
             print("KD:", self.angular_D,self.Kd )
             print("KI:", self.angular_I,self.Ki ) 
             self.move(0 , - angular_Correction )
             self.error_pub.publish(self.error)
             
             
             
        self.previous_time = self.current_time
        self.linear_previous_error  = linear_error 
        self.angular_previous_error = orien_error  
        
                
        
                                    #--------------UTILS FUNCTION--------------#
              
    def vector(self,dest_x,dest_y):
    
         bot_theta = self.odom.get_orientation("euler")["yaw"]      # bot making angle with x axis 
         bot_theta2 = self.odom.get_orientation("quaternion")
         bot_x_coordinate = round(self.odom.get_position()["x"], 2) # current x coordinate of bot
         bot_y_coordinate = round(self.odom.get_position()["y"],2)  # current y cordinate of bot
         bot_array = [np.cos(bot_theta),np.sin(bot_theta)]
         bot_vector = np.array(bot_array)
         reach_point_array = [dest_x - bot_x_coordinate , dest_y - bot_y_coordinate ]
         reach_point_vector = np.array(reach_point_array)
         dot_product = reach_point_vector.dot(bot_vector)
         bot_theta_error = np.arccos(dot_product/(np.sqrt((dest_x - bot_x_coordinate)**2+(dest_y - bot_y_coordinate)**2)))
         bot_position_error = np.sqrt((dest_x - bot_x_coordinate)**2+(dest_y - bot_y_coordinate)**2)
         if np.cos(bot_theta)*(dest_y - bot_y_coordinate) -  np.sin(bot_theta)*(dest_x - bot_x_coordinate) > 0 :
            bot_theta_error = - bot_theta_error 
            
         return [bot_vector,reach_point_vector,bot_theta_error, bot_position_error,bot_x_coordinate,bot_y_coordinate,bot_theta]
         
                   
                  
              
              
              
              
              
                                    #--------------CLASS CALLING--------------#  
                                           
curve = CurveTracer()
curve.control_loop()






