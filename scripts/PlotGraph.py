#!/usr/bin/env python3
import rospy
import time 
from curve_tracer.msg     import  PID_error_msg
import matplotlib.pyplot  as      plt
from matplotlib.widgets import Slider
from curve_tracer.msg import PID_constants


# GLOBAL VARIABLES 
#  `--> change here to tune settings of Graph 

# Setting Parameter       Descriptions 

Plot_width  = 110        # determines number of plot to show once at a time 
Upper_limit = 5        # maximum no to show on y-axis
Lower_limit = -5       # minimum no to show on y axis  
  
Time        = 0.1       # time after which graph will update itself 
Kp_Max = 10
Kp_Min = 0
Kd_Max = 0.01
Kd_Min = 0
Ki_Max = 10
Ki_Min = 0

Set_point   = 0        
Upper_precision_threshold = Set_point + rospy.get_param("curve_tracer_controller/theta_precision")
Lower_precision_threshold = Set_point - rospy.get_param("curve_tracer_controller/theta_precision")
axis_color  = 'lightgoldenrodyellow'

def callback(error):

       global error_linear , error_angular 
       
       error_linear = error.linear 
       error_angular = error.angular 
       
def Kp_changed(val):
     
     constant.proportional = val
     pub.publish(constant)
     

def Ki_changed(val):

     constant.integral = val
     pub.publish(constant)
     
    
def Kd_changed(val):

     constant.derivative = val
     pub.publish(constant)
     
    
    
def main():
  
  global error_linear , error_angular , Plot_width 
  global Upper_limit , Lower_limit , Time , initial_time
  global Kp_Min , Kp_Max , Ki_Max , Ki_Min , Kd_Max , Kd_Min
  

  
  
  # --- open up interactive frame 
  fig = plt.figure()
  plot_fig = fig.add_subplot(111)
  
  fig.subplots_adjust(left=0.10, bottom=0.30)
  
  plt.axhline(y=Set_point , color='black' ,linestyle='-',alpha=0.3)
  plt.axhline(y= Upper_precision_threshold  , color='green' ,linestyle='-', alpha=0.6)
  plt.axhline(y= Lower_precision_threshold  , color='green' ,linestyle='-',alpha=0.6)
  
  kp_slider_ax  = fig.add_axes([0.25, 0.15, 0.65, 0.03], facecolor=axis_color)
  kp_slider = Slider(kp_slider_ax, 'Kp', Kp_Min, Kp_Max, valinit=0)
  
  kd_slider_ax = fig.add_axes([0.25, 0.1, 0.65, 0.03], facecolor=axis_color)
  kd_slider = Slider(kd_slider_ax, 'Kd', Kd_Min, Kd_Max, valinit=0)
  
  ki_slider_ax = fig.add_axes([0.25, 0.05, 0.65, 0.03], facecolor=axis_color)
  ki_slider = Slider(ki_slider_ax, 'Ki', Ki_Min, Ki_Max, valinit=0)
  
  kp_slider.on_changed(Kp_changed)
  ki_slider.on_changed(Ki_changed)
  kd_slider.on_changed(Kd_changed)
  
  error_line, = plot_fig.plot ( time_axis , error_axis  )
  
  
  plot_fig.set_ylim([Lower_limit,Upper_limit])   
  fig.canvas.draw()
  plt.show(block=False)
  
  while not rospy.is_shutdown():
  
       try:
          
          
          for i in range((Plot_width)) :
          
               error_axis[i]  =  error_axis[i+1] 
               time_axis[i]   =  time_axis[i+1]
          error_axis[(Plot_width)] =  round(error_angular,2)
          time_axis[(Plot_width)]  =  time.time()-initial_time
         
          error_line.set_xdata(time_axis)
          error_line.set_ydata(error_axis)
          
          plot_fig.set_xlim([time.time()-initial_time-14,time.time()-initial_time + 0.2 ])
          plot_fig.relim() 
          plot_fig.autoscale_view(True,True,True) 
          
          
          fig.canvas.draw()
          plt.pause(0.0008)
          
       except KeyboardInterrupt :
           
        plt.close('all')
        break 
          

# -----Printing Information on Terminal 
print (" "*10+"\033[1m----------------------\033[0m")
print (" "*10+"\033[1m| PID Constant Tuner |\033[0m")
print (" "*10+"\033[1m----------------------\033[0m")

print ("To exit press ctrl + C")




# --- Initalizing list to plot Graph 
error_axis  , time_axis    = [0]*(Plot_width + 1) , [0]*(Plot_width + 1) 
upper_limit , lower_limit  = [Upper_limit]*(Plot_width + 1) , [Lower_limit]*(Plot_width + 1)
set_point =  [Set_point]*(Plot_width + 1)

# registering initial time             
initial_time = time.time()
   
# declaring 
error_linear  = 0
error_angular = 0

rospy.init_node('error_GraphPlot', anonymous=True)
rospy.Subscriber('/error_CurveTracer',PID_error_msg , callback)
  
pub = rospy.Publisher('/PID_constants', PID_constants, queue_size=10)
constant = PID_constants()
constant.proportional = 0
constant.integral = 0
constant.derivative = 0
pub.publish(constant)

main()


