#!/usr/bin/env python
import rospy
import numpy as np
from motor_operation.msg import set_point
from std_msgs.msg import Float32

# Setup Variables, parameters and messages to be used (if required)

start_time = 0.0
last_typeSignal = ""
last_step = 0.0
last_sinAmp = 0.0
last_squareAmp = 0.0



#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")

def checkingConditions():
  global stepSignal, squareAmp, sinAmp, maximumOmega, last_step, last_sinAmp, last_squareAmp
  if(stepSignal < -maximumOmega or stepSignal > maximumOmega ):
    stepSignal = last_step
  else:
    last_step = stepSignal
  
  if(sinAmp < -maximumOmega or sinAmp > maximumOmega):
    sinAmp = last_sinAmp
  else:
    last_sinAmp = sinAmp
  if(squareAmp < -maximumOmega or squareAmp > maximumOmega):
    squareAmp = last_squareAmp
  else:
    last_squareAmp = squareAmp
  if(squareAmp + squareOffset > maximumOmega or squareAmp + squareOffset < -maximumOmega):
    squareAmp = squareAmp - (squareAmp + squareOffset - maximumOmega)



if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("input")
    start_time = rospy.get_time()
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    pub = rospy.Publisher("set_point", set_point, queue_size = 10)
    mySetpoint = set_point()

    print("Input Signal Generator is Running")

    #Run the node
    while not rospy.is_shutdown():
      typeSignal = rospy.get_param("/typeSignal")
      stepSignal = rospy.get_param("/step")
      sinAmp = rospy.get_param("/sinAmp")
      sinPeriod = rospy.get_param("/sinPeriod")
      squarePeriod = rospy.get_param("/squarePeriod")
      squareAmp = rospy.get_param("/squareAmp")
      squareOffset = rospy.get_param("/squareOffset")
      maximumOmega = rospy.get_param("/maximumOmega")
      
      
      checkingConditions()
      
      time = rospy.get_time() - start_time
      if(typeSignal == "sin"):
        mySetpoint.data = sinAmp * np.sin(time/sinPeriod) 
        last_typeSignal = "sin"                
      elif(typeSignal == "square"):
        mySetpoint.data = squareAmp*np.sign(np.sin( np.pi * 1 * time / squarePeriod)) + squareOffset
        last_typeSignal = "square"
      elif(typeSignal == "step"):                                                                                                   
        mySetpoint.data = stepSignal
        last_typeSignal = "step"
      else:
        mySetpoint.data = last_typeSignal
      mySetpoint.time = rospy.get_time() - start_time
      #Write your code here 
      pub.publish(mySetpoint)

      rate.sleep()
