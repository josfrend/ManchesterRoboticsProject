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
  global last_error, last_time, integral
  current_time = time
  dt = current_time - last_time

  error = setpoint - feedback
  derivative = (error - last_error)
  integral += error * dt

  output = kp * error + ki * integral + kd * derivative
  last_error = error
  last_time = current_time

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
        pub.publish(adjustment)
        rate.sleep()