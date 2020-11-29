#!/usr/bin/env python

import rospy
import time
from pybot_scara.msg import pybot_msg  # from <packge_name> import <custom_message_name>
from PyBotArm import PyBotArm # Import Class to control the ROBOT

# Robot initialization
scara = PyBotArm() # the robot is called "scara"
status = scara.connect("USB","/dev/ttyACM0") # Serial (USB) connection

if (status):
    print("Robot well connected and ready to start.")
else:
    print("Robot not well connected.")
    exit()

def subscriber():

    sub = rospy.Subscriber('move_real_pybot', pybot_msg, callback_function) # initialize the subscriber object
    rospy.init_node('xyz_subscriber', anonymous=True)
    rospy.spin()

# callback_function: is a function that is called every time a message is published to the topic
def callback_function(message):

    # Definition of Robots' neutral position and speed
    def neutral_pose():
        print("Robot neutral position")
        scara.moveAllAxis(0,0,0)
        scara.setSpeedAcc(50,50,50)

    # Definition of points to reach:
    eef_pose = True # flag
    if (eef_pose):
        print("Position described by the publisher")
        scara.moveXYZ(message.num_float[0],message.num_float[1],message.num_float[2],elbow=1,t=1)
        rospy.loginfo(message.num_float) # to print the published message in the terminal

        # Back to neutral position
        neutral_pose()


if __name__ == "__main__":
    subscriber()
