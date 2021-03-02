#!/usr/bin/env python

from __future__ import print_function
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
 
#### config
odom_topic = 'pose'
#####

roll = pitch = yaw = 0.0

def get_rotation(msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

rospy.init_node('show_angle')
r = rospy.Rate(10)
sub = rospy.Subscriber (odom_topic, Odometry, get_rotation)

while not rospy.is_shutdown():
        yaw_deg = yaw*180/math.pi
        print("current angle: {}".format(yaw_deg))
        r.sleep()