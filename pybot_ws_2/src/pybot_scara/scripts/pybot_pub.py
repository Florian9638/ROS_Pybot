#!/usr/bin/env python

import rospy
from pybot_scara.msg import pybot_msg  # from <packge_name> import <custom_message_name>

def publisher():

    pub = rospy.Publisher('move_real_pybot', pybot_msg, queue_size=10) # topic name and message type
    rospy.init_node('xyz_publisher', anonymous=True) # initialize node
    rate = rospy.Rate(5)

    msg_to_publish = pybot_msg() # The msg_to_publish will contain the x,y,z coordinates of the end-effector. pybot_msg is the custom message file I created


    while not rospy.is_shutdown():
        # Desired coordinates of the end-effector
        #xyz = [0.0, 110.0, 0.0]
        xyz = input('Enter float eef coordinates [x, y, z]: ')

        msg_to_publish.num_float = xyz # num_float is the name of a desired float32[] array (list) that I defined in the pybot_msg.msg file. I define it now as [x,y,z] coordinates

        pub.publish(msg_to_publish) # what this node is going to publish

        rospy.loginfo(xyz) # to print the published message in the terminal

        rate.sleep()

if __name__ == '__main__':
    publisher()
