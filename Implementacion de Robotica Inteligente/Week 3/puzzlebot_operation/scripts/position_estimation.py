#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from puzzlebot_operation.msg import goal

class puzzlebot:
    def __init__(self):
        # Initialize variables for wheel speeds, robot pose, target pose, and error values
        self.wr = 0.0
        self.wl = 0.0
        self.r = 0.05
        self.l = 0.19
        self.x_k = 0.0
        self.y_k = 0.0
        self.target_x = 0.0
        self.target_y = 0.0 
        self.theta_k = 0.0
        self.error_d = 0.0
        self.error_theta = 0.0
        self.last_error_theta = 0.0
        self.last_error_d = 0.0
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
        self.kp_angular = 0.54
        self.kp_linear = 0.34
        self.ki_angular = 0.0
        self.integral_angular = 0.0

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
        rospy.Subscriber("/goal", goal, self.callbackGoal)
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
        if(output > 0 and output < 0.05): output = 0.05
        elif(output > -0.05 and output < 0): output = -0.05
        return output
    
    def PID_Linear(self):
        #PID controller for linear velocity
        output = self.kp_linear * self.error_d
        self.last_error_d = self.error_d
        return output

    def callbackWr(self, msg):
        self.wr = msg.data
    
    def callbackWl(self, msg):
        self.wl = msg.data
    
    def callbackGoal(self,msg):
        self.target_x = msg.pos_x
        self.target_y = msg.pos_y

    def stop(self):
        #On exit stop the puzzlebot
        self.speed.angular.z = 0.0
        self.speed.linear.x = 0.0
        self.main_pub.publish(self.speed)
        print("Stopping")

    def run(self):
        angulo_deseado = 0.0

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

            self.error_theta = (np.arctan2(self.target_y - self.y_k, self.target_x - self.x_k)) - self.theta_k
            self.error_d = np.sqrt((self.target_x-self.x_k)**2 + (self.target_y-self.y_k)**2)
            self.error_theta = self.wrapTheta(self.error_theta)
            self.data_out.x = self.x_k
            self.data_out.y = self.y_k
            self.data_out.theta = self.theta_k

           
             # Control logic for robot movement
        
            # If the angular error is greater than a threshold and the angular threshold is not reached yet
            if(np.rad2deg(abs(self.error_theta)) > 0.3 and self.angular_threshold_reached == False):
                self.speed.linear.x = 0.0
                self.speed.angular.z = self.PID_Angular()
                print("I'm moving angularly")
            
            # If the angular error is less than a threshold and the pause counter is less than 50 stop the robot
            elif(np.rad2deg(abs(self.error_theta)) < 0.3 and self.pause < 50):
                self.pause += 1
                self.angular_threshold_reached = True
                self.speed.linear.x = 0.
                self.speed.angular.z = 0.

            # If the linear error is greater than a threshold and the linear threshold is not reached yet
            elif(self.error_d > 0.1 and self.linear_threshold_reached == False):
                self.speed.linear.x = self.PID_Linear()
                self.speed.angular.z = 0.
                self.angular_threshold_reached = True
                print("I'm moving linearly")
                if(self.error_d > 4):
                    self.linear_threshold_reached = True

            # If none of the above conditions are met, stop the robot and reset relevant variables   
            else:
                self.speed.linear.x = 0.
                self.speed.angular.z = 0.
                if(self.target_x != self.last_target[0] or self.target_y != self.last_target[1]):
                    self.linear_threshold_reached = False
                    self.angular_threshold_reached = False
                    self.hold = True
                self.pause = 0
                self.integral_angular = 0
                self.last_target = [self.target_x, self.target_y]

             # Publish robot's speed, pose, and error values
            self.main_pub.publish(self.speed)
            self.pub.publish(self.data_out)
            self.pub2.publish(np.rad2deg(self.error_theta))
            self.pub3.publish(self.error_d)
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
