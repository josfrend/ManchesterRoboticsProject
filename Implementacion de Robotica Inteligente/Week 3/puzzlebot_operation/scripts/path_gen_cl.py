#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Float32
from puzzlebot_operation.msg import goal

# Setup Variables


time = 0
error_theta = 99999999.0
error_d = 99999999.0
pointSelf = [0,0]



def callbackErrorTheta(msg):
   global error_theta
   error_theta = msg.data

def callbackErrorD(msg):
   global error_d
   error_d = msg.data

#Stop Condition
def stop():
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("path_generator")
    start_time = rospy.get_time()
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers 
    pub = rospy.Publisher("/goal", goal, queue_size = 2)
    pub2 = rospy.Publisher("/balance", Float32, queue_size= 10)
    rospy.Subscriber("/error_theta", Float32, callbackErrorTheta)
    rospy.Subscriber("/error_d", Float32, callbackErrorD)
    goal = goal()
    input_path = rospy.get_param("/path")
    path = []

    check = 0
    step = 0
    balance = 0.0
    time_elapse = 0.0
    

    print("Path Generator is Running")
    

    #Run the node
    while not rospy.is_shutdown():
        if(check == 0):
            for i in range(0,len(input_path)/2):
                path.append([input_path[i*2],input_path[i*2+1]])
            print(path)
            start_time = rospy.get_time()
            time_elapse = start_time
        else:
            time = rospy.get_time() - start_time
            # Update step when error_d is less than 0.1 sec and it has been for at least 0.2 sc
            # then reset balance with a sleep of 0.1 sec
            if(error_d < 0.1 and step < len(path)-1 ):
              balance += rospy.get_time() - time_elapse
              if(balance > 0.2):
                step += 1
                balance = 0.0
                rospy.sleep(0.1)
              start_time = rospy.get_time()
            else:
              balance = 0.0

            time_elapse = rospy.get_time()
               
        
        goal.pos_x = path[step][0]
        goal.pos_y = path[step][1]
        check +=1
      
        pub.publish(goal)
        pub2.publish(balance)
      
      #Write your code here 
      

        rate.sleep()
