#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler



class OdomSubscriber():
     rospy.init_node('OdomSubscriber',anonymous=False)
     topic = '/odom'

     def init(self):
         rospy.Subscriber(self.topic, Odometry, self.odom_callback)
         self.odom_data = Odometry()

     def odom_callback(self, msg):
         self.odom_data = msg

     def get_position(self):
         odom_position = {"x" :self.odom_data.pose.pose.position.x, "y" :self.odom_data.pose.pose.position.y, "z" :self.odom_data.pose.pose.position.z}
         return (Odom_position)

     def get_orientation(self,orientation_choice="quaternion"):
         odom_orientation_quaternion= {"w" :self.odom_data.pose.pose.orientation.w, "x" :self.odom_data.pose.pose.orientation.x , "y" :self.odom_data.pose.pose.orientation.y, "z" :   self.odom_data.pose.pose.orientation.z}
         x  = self.odom_data.pose.pose.orientation.x;
         y  = self.odom_data.pose.pose.orientation.y;
         z = self.odom_data.pose.pose.orientation.z;
         w = self.odom_data.pose.pose.orientation.w;


         (roll, pitch, yaw) = euler_from_quaternion ([x,y,w,z])
         odom_orientation_euler = {'roll':roll,'pitch':pitch,'z':yaw}
         if orientation_choice.lower() == 'euler':
            return odom_orientation_euler
         else :
            return odom_orientation_quaternion





