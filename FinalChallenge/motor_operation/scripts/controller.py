#!/usr/bin/env python
import rospy
import numpy as np
from motor_operation.msg import set_point
from std_msgs.msg import Float32
referencia = 0.0
time = 0.0
salida = 0.0
estado = ""

adjustment = Float32()
error_val = Float32()
error_kp = Float32()
error_ti = Float32()
error_td = Float32()
set_up_delay = 0
kp = rospy.get_param("/system_param_Kp")
ki = rospy.get_param("/system_param_Ti")
kd = rospy.get_param("/system_param_Kd")

integral = 0.0
last_error = 0.0
last_time = 0.0

#Setup parameters, vriables and callback functions here (if required)
def callback(msg):
  global salida, estado
  salida = msg.data

def callback2(msg):
  global referencia, time
  referencia = msg.data
  time = msg.time


def PID_Discrete(feedback, setpoint):
  global last_error, last_time, integral, error_val, error_kp, error_td, error_ti
  current_time = time
  dt = current_time - last_time

  error = setpoint - feedback
  derivative = (error - last_error)
  integral += error * dt

  output = kp * error + ki * integral + kd * derivative
  last_error = error
  last_time = current_time
  error_val.data = error
  error_kp.data = kp * error
  error_ti.data = ki * integral
  error_td.data = kd * derivative

  return output



#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("controller")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)
    last_time = time


    #Setup Publishers and subscribers here
    pub = rospy.Publisher("motor_input", Float32, queue_size = 10)
    pub2 = rospy.Publisher("error", Float32, queue_size = 10)
    pub3 = rospy.Publisher("errorKp", Float32, queue_size = 10)
    pub4 = rospy.Publisher("errorKi", Float32, queue_size = 10)
    pub5 = rospy.Publisher("errorKd", Float32, queue_size = 10)
    rospy.Subscriber("motor_output", Float32, callback)
    rospy.Subscriber("set_point", set_point, callback2)



    print("The Controller is Running")
    #Run the node
    while not rospy.is_shutdown():

        #Write your code here
        adjustment.data = PID_Discrete(salida, referencia)
        if(adjustment.data > 1.0):
            adjustment.data = 1.0
        elif(adjustment.data < -1.0):
            adjustment.data = -1.0

        #adjustment.data = referencia
        
        pub.publish(adjustment)
        pub2.publish(error_val)
        pub3.publish(error_kp)
        pub4.publish(error_ti)
        pub5.publish(error_td)
        rate.sleep()