#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
import numpy as np
import math

class Waypoint_Mananger:

  def __init__(self,res=1):
    self.index = 0
    self.res = res
    self.way_point()

  def way_point(self):

    #create a numpy array of 'res' points in range 0 to 2*pi
    x = np.linspace(0, 2*np.pi, num = self.res, endpoint=True)[1:]
    #create a numpy array of given path function 
    y = np.sin(x)
    #Returning waypoints in form of (x,y)
    self.list_waypoints  = list(zip(x,y))
    
    return self.list_waypoints
  
  def get_next_waypoint(self):
     
    #Checking if the bot has reached the destination
    if self.index >= len(self.list_waypoints):
      print("Bot has reached its destination")
      return None 
    
    self.next_waypoint = self.list_waypoints[self.index]
    self.index = self.index + 1
  
    return self.next_waypoint
