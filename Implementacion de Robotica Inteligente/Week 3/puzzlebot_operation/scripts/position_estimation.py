#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

class puzzlebot:
    def __init__(self):
        self.wr = 0.0
        self.wl = 0.0
        self.r = 0.05
        self.l = 0.19
        self.x_k = 0.0
        self.y_k = 0.0
        self.theta_k = 0.0
        self.error_d = 0.0
        self.error_theta = 0.0
        self.last_error_theta = 0.0
        self.last_error_d = 0.0
        self.total_distance = 0.0
        self.total_angle = 0.0
        self.speed = Twist()
        self.speed.linear.x = 0.0
        self.speed.linear.y = 0.0
        self.speed.linear.z = 0.0
        self.speed.angular.x = 0.0
        self.speed.angular.y = 0.0
        self.speed.angular.z = 0.0
        self.beginning = False
        self.current_step = 0 
        self.check = 0
        self.angular_threshold_reached = False
        self.linear_threshold_reached = False

        self.kp_angular = 0.49
        self.kp_linear = 0.4

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
        self.start_time = rospy.get_time()
        print("Position node is running")
    
    def wrapTheta(self):
        if(self.theta_k > np.pi or self.theta_k <= -np.pi):
            self.theta_k = (self.theta_k + np.pi) % (2 * np.pi) - np.pi

    def PID_Angular(self):
        output = self.kp_angular * self.error_theta
        self.last_error_theta = self.error_theta
        return output
    
    def PID_Linear(self):
        output = self.kp_linear * self.error_d
        self.last_error_d = self.error_d
        return output

    def callbackWr(self, msg):
        self.wr = msg.data
    
    def callbackWl(self, msg):
        self.wl = msg.data
    
    def stop(self):
        print("Stopping")

    def run(self):
        target_position = [[1,0], [1,1],[0,1],[0,0]]

        self.dt = rospy.get_time() - self.start_time

        if(self.beginning == False):
            self.dt = 0.0

        if(self.angular_threshold_reached == False):
            self.theta_k = self.theta_k + (self.r*((self.wr - self.wl) /self.l)) * self.dt
        self.wrapTheta()
        self.x_k = self.x_k + (self.r*(self.wr + self.wl) /2) * self.dt * np.cos(self.theta_k)
        self.y_k = self.y_k + (self.r*(self.wr + self.wl) /2) * self.dt * np.sin(self.theta_k)

        self.error_theta = (np.arctan2(target_position[self.current_step][1]-self.y_k, target_position[self.current_step][0]-self.x_k)) - self.theta_k
        self.error_d = np.sqrt((target_position[self.current_step][0]-self.x_k)**2 + (target_position[self.current_step][1]-self.y_k)**2)

        self.data_out.x = self.x_k
        self.data_out.y = self.y_k
        self.data_out.theta = self.theta_k



        if(np.rad2deg(abs(self.error_theta)) > 0.1 and self.angular_threshold_reached == False):
            self.speed.linear.x = 0.0
            self.speed.angular.z = self.PID_Angular()

        elif(self.error_d > 0.05 and self.linear_threshold_reached == False):
            self.speed.linear.x = self.PID_Linear()
            self.speed.angular.z = 0.
            self.angular_threshold_reached = True
            #self.error_theta = 0.0
            if(self.error_d > 4):
                self.linear_threshold_reached = True
                
        else:
            self.linear_threshold_reached = False
            self.angular_threshold_reached = False
            self.speed.linear.x = 0.
            self.speed.angular.z = 0.
            self.error_theta = 0.
            if(self.current_step < len(target_position)-1):
                self.current_step += 1
            #print("im here")

        #print(self.angular_threshold_reached)
        self.main_pub.publish(self.speed)
        self.pub.publish(self.data_out)
        self.pub2.publish(np.rad2deg(self.error_theta))
        self.pub3.publish(self.error_d)
        self.start_time = rospy.get_time()
        if(self.check > 20):
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
