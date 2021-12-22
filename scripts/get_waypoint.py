#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
import numpy as np
import math

class WaypointManager:

  def __init__(self,res=1,index=0,list_x=[],list_y=[]):
    self.res = res
    self.index = index
    self.list_x = list_x
    self.list_y = list_y

  def way_point(self):

    #create a numpy array of 'res' points in range 0 to 2*pi
    x = np.linspace(0, 2*np.pi, num = self.res, endpoint=True)[1:]
    #create a numpy array of given path function 
    y = np.sin(x)

    #Returning waypoints in form of (x,y)    
    return list(zip(x,y))
