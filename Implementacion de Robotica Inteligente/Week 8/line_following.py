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
        self.last_error_d = 0.0
        self.last_target = [0,0]
        self.true_output = 0.

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
        self.kp_angular = 0.52#0.44#0.54
        self.kp_linear = 0.34
        self.ki_angular = 0.15
        self.kd_angular = 0.
        self.integral_angular = 0.0
        
        # Store light color 
        self.light_color = "n"

        # Initialize ROS publishers for robot speed and pose, and subscribers for wheel speeds and goal pose
        self.main_pub = rospy.Publisher("/cmd_vel", Twist, queue_size= 10)
        rospy.Subscriber("/wr", Float32, self.callbackWr)
        rospy.Subscriber("/wl", Float32, self.callbackWl)
        rospy.Subscriber("/alignment", Int32, self.callbackAlignment)
        rospy.Subscriber("/light_color", String, self.callbackColor)
        self.start_time = rospy.get_time()
        print("Line following node is running")
    
    def wrapTheta(self, angle):
        #Keep the input angle between values of -pi and pi
        if(angle > np.pi or angle <= -np.pi):
            angle = (angle + np.pi) % (2 * np.pi) - np.pi
        return angle

    
    def PID_Following(self):
        max_angular = 0.03
        #PID controller for line following
        self.integral_angular += self.line_error * self.dt
        derivative = (self.line_error - self.last_error_theta)
        output = self.kp_angular * self.line_error + self.ki_angular * self.integral_angular + self.kd_angular * derivative
        self.last_error_theta = self.line_error
        self.true_output = output
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

            # Update robot's pose 
            self.theta_k = self.theta_k + (self.r*((self.wr - self.wl) /self.l)) * self.dt
            self.theta_k = self.wrapTheta(self.theta_k)
            self.x_k = self.x_k + (self.r*(self.wr + self.wl) /2) * self.dt * np.cos(self.theta_k)
            self.y_k = self.y_k + (self.r*(self.wr + self.wl) /2) * self.dt * np.sin(self.theta_k)

            # Adjust the linear speed according to the color identified 
            desired_speed = 0.02
            if(self.light_color == "g"): actual_speed = desired_speed
            elif(self.light_color == "y"): actual_speed = desired_speed/2
            elif(self.light_color == "r"): actual_speed = 0
            else: actual_speed = desired_speed
            self.speed.linear.x = actual_speed
            
            # Use a PID controller to adjust angular speed if the line is outside of the threshold
            if(abs(self.line_error) > 15):
                self.speed.angular.z = self.PID_Following()
            else:
                self.speed.angular.z = 0.
 
            

             # Publish robot's speed, pose, and error values
            self.main_pub.publish(self.speed)
            self.pub2.publish(np.rad2deg(self.error_theta))
            self.pub3.publish(self.true_output)
            self.start_time = rospy.get_time()

        if(self.check > 2 ):
            self.beginning = True
        self.check += 1


if __name__ == '__main__':
    rospy.init_node("position_estimation")
    
    robot = puzzlebot()
    rate = rospy.Rate(100)
    rospy.on_shutdown(robot.stop)

    try:
        while not rospy.is_shutdown():
            robot.run()
            rate.sleep()
    except rospy.ROSInterruptException():
        pass
