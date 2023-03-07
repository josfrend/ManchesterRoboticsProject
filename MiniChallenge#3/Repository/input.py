#!/usr/bin/env python
# The above line specifies the path to the Python interpreter

# Import the required modules and message types
import rospy
import numpy as np
from pid_control.msg import set_point
from std_msgs.msg import Int16

# Set up variables and messages to be used
start_time = 0.0

# Define a function to stop the node
def stop():
    # This function will be called when the node is terminated
    print("Stopping")

if __name__=='__main__':
    # Initialize the ROS node with the name "input"
    rospy.init_node("input")

    # Get the current time to use as the start time
    start_time = rospy.get_time()

    # Set the rate at which messages will be published (100 Hz)
    rate = rospy.Rate(100)

    # Call the "stop" function when the node is terminated
    rospy.on_shutdown(stop)

    # Set up publishers for the "cmd_pwm" and "setpoint" topics
    pub = rospy.Publisher("cmd_pwm", Int16, queue_size=10)
    pub2 = rospy.Publisher("setpoint", Int16, queue_size=10)

    # Initialize an "Int16" message for the setpoint topic
    mySetpoint = Int16()

    # Initialize another "Int16" message to graph the setpoint
    mySetpoint2 = Int16()
    mySetpoint2.data = 255

    # Print a message to indicate that the node is running
    print("Input Signal Generator is Running")

    # Run the main loop until the node is terminated
    while not rospy.is_shutdown():
        # Get the value of the "typeSignal" parameter
        typeSignal = rospy.get_param("/typeSignal")

        # Calculate the current time since the node started
        time = rospy.get_time() - start_time

        # Generate the setpoint value based on the "typeSignal" parameter
        if typeSignal == 1:
            # Sine wave with a frequency of 0.2 Hz and an amplitude of 250
            mySetpoint.data = 250 * np.sin(time / 5) + 255
        elif typeSignal == 2:
            # Square wave with a frequency of 0.2 Hz and an amplitude of 140
            mySetpoint.data = 140 * np.sign(np.sin(np.pi * 1 * time / 5)) + 140
        else:
            # Constant setpoint value of 390
            mySetpoint.data = 390

        # Publish the setpoint message
        pub.publish(mySetpoint)

        # Publish the second setpoint message (not used in this script)
        pub2.publish(mySetpoint2)

        # Wait until it's time to publish the next message
        rate.sleep()
