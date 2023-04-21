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
        self.error_theta = 0.0
        self.theta_k = 0.0
        self.total_distance = 0.0
        self.total_angle = 0.0
        self.speed = Twist()
        self.speed.linear.x = 0.0
        self.speed.linear.y = 0.0
        self.speed.linear.z = 0.0
        self.speed.angular.x = 0.0
        self.speed.angular.y = 0.0
        self.speed.angular.z = 0.0
        self.firstIteration = False

        self.pub = rospy.Publisher("position", Pose2D, queue_size = 10)
        self.pub2 = rospy.Publisher("error_theta", Float32, queue_size = 10)
        self.data_out = Pose2D()
        self.data_out.x = 0.0
        self.data_out.y = 0.0
        self.data_out.theta = 0.0
        rospy.Subscriber("/wr", Float32, self.callbackWr)
        rospy.Subscriber("/wl", Float32, self.callbackWl)
        self.start_time = rospy.get_time()
        print("Position node is running")
    
    def wrapTheta(self):
        if(self.theta_k > math.pi):
            self.theta_k -= 2 * math.pi
        elif(self.theta_k <= math.pi):
            self.theta_k += 2 * math.pi 

    def callbackWr(self, msg):
        self.wr = msg.data
    
    def callbackWl(self, msg):
        self.wl = msg.data
    
    def stop(self):
        print("Stopping")

    def run(self):
        target_position = [2,2]

        self.dt = rospy.get_time() - self.start_time

        if(self.firstIteration == False):
            self.dt = 0.0
    
        self.theta_k = self.theta_k + (self.r*((self.wr - self.wl) /self.l)) * self.dt
        self.x_k = self.x_k + (self.r*(self.wr + self.wl) /2) * self.dt * np.cos(self.theta_k)
        self.y_k = self.y_k + (self.r*(self.wr + self.wl) /2) * self.dt * np.sin(self.theta_k)

        self.error_theta = (np.arctan2(target_position[0]-self.x_k, target_position[1]-self.y_k)) - self.theta_k

        self.data_out.x = self.x_k
        self.data_out.y = self.y_k
        self.data_out.theta = self.theta_k
        self.wrapTheta()

        self.pub.publish(self.data_out)
        self.pub2.publish(self.error_theta)
        self.start_time = rospy.get_time()
        self.firstIteration = True


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