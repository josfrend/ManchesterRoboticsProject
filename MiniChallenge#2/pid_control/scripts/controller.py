#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import motor_output
from pid_control.msg import motor_input
from pid_control.msg import set_point
referencia = 0.0
time = 0.0
salida = 0.0
estado = ""

adjustment = motor_input()
set_up_delay = 0
kp = rospy.get_param("/system_param_Kp")
ti = rospy.get_param("/system_param_Ti")

#Setup parameters, vriables and callback functions here (if required)
def callback(msg):
  global salida, estado
  salida = msg.output
  estado = msg.status

def callback2(msg):
  global referencia, time
  referencia = msg.reference
  time = msg.time

e = 0.0
e_1 = 0.0
u = 0.0
u_1 = 0.0

q0 = 0.0
q1 = 0.0

def PID_Initialize():
    global kp, ti, q0, q1
    
    t = 0.01    #sample rate
    
    # Calculate the coefficients for the discrete PID controller
    #kp = (0.9 * tao) / (k * theta)
    #ti = 3.33 * theta
    q0 = kp + ((kp * t)/(2.0 * ti))
    q1 = ((kp * t)/(2.0 * ti)) - kp

def PID_Discrete(yM, max_val, min_val, set_point):
    global e, e_1, u, u_1
    
    # Calculate the error
    e = set_point - yM
    
    # Calculate the discrete PID controller
    u = u_1 + q0 * e + q1 * e_1
    
    # Saturate the controller with upper and lower limits
    if u >= max_val:
        u = max_val
    elif u <= min_val:
        u = min_val
    
    # Update the values for the next iteration
    e_1 = e
    u_1 = u
    
    return u
  

#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("controller")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)
    PID_Initialize()

    #Setup Publishers and subscribers here
    pub = rospy.Publisher("motor_input", motor_input, queue_size = 10)
    rospy.Subscriber("motor_output", motor_output, callback)
    rospy.Subscriber("set_point", set_point, callback2)



    print("The Controller is Running")
    #Run the node
    while not rospy.is_shutdown():

        #Write your code here
      if(set_up_delay >= 1000): adjustment.input = PID_Discrete(salida, 13.0006322861, -13.0006322861, referencia)
      else: adjustment.input = 0.0
      adjustment.time = time
      pub.publish(adjustment)
      set_up_delay += 1
      rate.sleep()