#!/usr/bin/env python    
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose     


class OdomSubscriber():
     rospy.init_node('OdomSubscriber',anonymous=False)
     topic = '/odom'
     
     def __init__(self):
         rospy.Subscriber(self.topic, Odometry, self.odom_callback)  
         self.odom_data = Odometry()
        
     def odom_callback(self, msg):
         self.odom_data = msg
      
     def get_position(self):
         odom_position = {"x" :self.odom_data.pose.pose.position.x, "y" :self.odom_data.pose.pose.position.y, "z" :self.odom_data.pose.pose.position.z}
         return (Odom_position)
         
     def get_orientation(self):
         odom_orientation = {"w" :self.odom_data.pose.pose.orientation.w, "x" :self.odom_data.pose.pose.orientation.x , "y" :self.odom_data.pose.pose.orientation.y, "z" : self.odom_data.pose.pose.orientation.z}
         return (odom_orientation)
         


call_odomsubs = OdomSubscriber()
while(True):
  print(call_odomsubs.get_orientation()['x'])
rospy.spin()

