#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

class puzzlebot:
    def __init__(self): 
        # Initialize variables for wheel speeds, robot pose, target pose, and error values
        self.wr = 0.0
        self.wl = 0.0
        self.r = 0.05
        self.l = 0.19
        self.x_k = 0.0
        self.y_k = 0.0
        self.line_error = 0
        self.theta_k = 0.0
        self.error_d = 0.0
        self.error_theta = 0.0
        self.last_error_theta = 0.0
        self.last_error_follow = 0.0
        self.last_error_d = 0.0
        self.trajectory = []
        self.target_x = 0.
        self.target_y = 0.
        self.last_target = [0,0]

        # Initialize Twist message for robot speed
        self.speed = Twist()
        self.speed.linear.x = 0.0
        self.speed.linear.y = 0.0
        self.speed.linear.z = 0.0
        self.speed.angular.x = 0.0
        self.speed.angular.y = 0.0
        self.speed.angular.z = 0.0

        # Initialize flags and parameters for control logic
        self.beginning = False
        self.current_step = 0 
        self.check = 0
        self.pause = 0
        self.hold = False
        self.angular_threshold_reached = False
        self.linear_threshold_reached = False

        # Initialize PID controller gains and integrals
        self.kp_follow = 0.0038#0.0015
        self.ki_follow = 0.00#0.38
        self.kd_follow = 0.0005#0.00005
        self.kp_angular = 0.48
        self.ki_angular = 0.52
        self.kp_linear = 0.34
        self.integral_angular = 0.0
        self.integral_follow = 0.0

        self.light_color = "n"
        self.sign = ""
        self.sign_inst_completed = True

        # Initialize ROS publishers for robot speed and pose, and subscribers for wheel speeds and goal pose
        self.main_pub = rospy.Publisher("/cmd_vel", Twist, queue_size= 10)
        self.pub = rospy.Publisher("position", Pose2D, queue_size = 10)
        self.pub2 = rospy.Publisher("error_theta", Float32, queue_size = 10)
        self.pub3 = rospy.Publisher("error_d", Float32, queue_size = 10)
        self.data_out = Pose2D()
        self.data_out.x = 0.0
        self.data_out.y = 0.0
        self.data_out.theta = 0.0
        rospy.Subscriber("/wr", Float32, self.callbackWr)
        rospy.Subscriber("/wl", Float32, self.callbackWl)
        rospy.Subscriber("/alignment", Int32, self.callbackAlignment)
        rospy.Subscriber("/light_color", String, self.callbackColor)
        rospy.Subscriber("/sign", String, self.callbackSign)
        self.start_time = rospy.get_time()
        print("Position node is running")
    
    def wrapTheta(self, angle):
        #Keep the input angle between values of -pi and pi
        if(angle > np.pi or angle <= -np.pi):
            angle = (angle + np.pi) % (2 * np.pi) - np.pi
        return angle

    def PID_Angular(self):
        #PID controller for angular velocity
        self.integral_angular += self.error_theta * self.dt
        output = self.kp_angular * self.error_theta + self.ki_angular * self.integral_angular
        self.last_error_theta = self.error_theta
        if(output > 0 and output < 0.18): output = 0.18
        elif(output > -0.18 and output < 0): output = -0.18
        return output
    
    def PID_Linear(self):
        #PID controller for linear velocity
        max_lineal = 0.07
        output = self.kp_linear * self.error_d
        self.last_error_d = self.error_d
        if(output > max_lineal): output = max_lineal
        return output
    
    def PID_Following(self):
        max_angular = 1.0#0.15
        #PID controller for angular velocity
        self.integral_follow+= self.line_error * self.dt
        derivative = (self.line_error - self.last_error_theta)
        output = self.kp_follow * self.line_error + self.ki_follow * self.integral_follow + self.kd_follow * derivative
        self.last_error_follow = self.line_error
        if(output > max_angular): output = max_angular
        elif(output < -max_angular): output = -max_angular
        return output

    def callbackWr(self, msg):
        self.wr = msg.data
    
    def callbackWl(self, msg):
        self.wl = msg.data
    
    def callbackAlignment(self,msg):
        self.line_error = -0.5*msg.data

    def callbackColor(self,msg):
        self.light_color = msg.data

    def callbackSign(self,msg):
        self.sign = msg.data

    def stop(self):
        #On exit stop the puzzlebot
        self.speed.angular.z = 0.0
        self.speed.linear.x = 0.0
        self.main_pub.publish(self.speed)
        print("Stopping")

    def run(self):

        self.dt = rospy.get_time() - self.start_time

        if(self.beginning == False):
            print("Initializing")
        else:
            if(self.hold == True):
                rospy.sleep(0.1)
                self.hold = False

            # Update robot's pose and error values
            self.theta_k = self.theta_k + (self.r*((self.wr - self.wl) /self.l)) * self.dt
            self.theta_k = self.wrapTheta(self.theta_k)
            self.x_k = self.x_k + (self.r*(self.wr + self.wl) /2) * self.dt * np.cos(self.theta_k)
            self.y_k = self.y_k + (self.r*(self.wr + self.wl) /2) * self.dt * np.sin(self.theta_k)

            

           
             # Control logic for robot movement
        
            # If the linear error is greater than a threshold and the linear threshold is not reached yet
            
            desired_speed = 0.1
            if(self.light_color == "g"): actual_speed = desired_speed
            elif(self.light_color == "y"): actual_speed = desired_speed/2
            elif(self.light_color == "r"): actual_speed = 0
            else: actual_speed = desired_speed
            self.speed.linear.x = actual_speed

            if(abs(self.line_error) > 15 and self.sign_inst_completed):
                self.speed.angular.z = self.PID_Following()
                self.theta_k = 0.
                self.x_k = 0.
                self.y_k = 0.
                print("Following the line")
            else:
                self.speed.angular.z = 0.

                if(self.sign != ""):
                    self.sign_inst_completed = False
                    if(self.sign == "left"):
                        self.target_x = 1
                        self.target_y = 1
                        
                        
                    self.error_theta = (np.arctan2(self.target_y - self.y_k, self.target_x - self.x_k)) - self.theta_k
                    self.error_d = np.sqrt((self.target_x-self.x_k)**2 + (self.target_y-self.y_k)**2)
                    self.error_theta = self.wrapTheta(self.error_theta)
                    if(self.pause < 100):
                        self.speed.linear.x = self.PID_Linear()
                        self.speed.angular.z = 0.
                        print("I'm moving linearly")
                        self.pause +=1

                    if(np.rad2deg(abs(self.error_theta)) > 0.3 and self.error_d > 0.1):
                        self.speed.linear.x = self.PID_Linear()
                        self.speed.angular.z = self.PID_Angular()
                        print("I'm moving")
                    else:
                        self.sign_inst_completed = True
                        self.sign = ""
                        self.pause = 0



                

            # self.speed.angular.z = 0.
                

            # If none of the above conditions are met, stop the robot and reset relevant variables   
            

             # Publish robot's speed, pose, and error values
            self.main_pub.publish(self.speed)
            self.pub2.publish(np.rad2deg(self.error_theta))
            #self.pub3.publish(self.true_output)
            self.start_time = rospy.get_time()

        if(self.check > 2 ):
            self.beginning = True
        self.check += 1


if __name__ == '__main__':
    rospy.init_node("line_following")
    
    robot = puzzlebot()
    rate = rospy.Rate(100)
    rospy.on_shutdown(robot.stop)

    try:
        while not rospy.is_shutdown():
            robot.run()
            rate.sleep()
    except rospy.ROSInterruptException():
        pass
