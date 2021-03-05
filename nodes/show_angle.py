#!/usr/bin/env python
from config import Config
import math
from nav_msgs.msg import Odometry
import rospy
import sys
from tf.transformations import euler_from_quaternion, quaternion_from_euler

roll = pitch = yaw = 0.0

def get_rotation(msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

if __name__=="__main__":
    # get odom_topic
    if len(sys.argv) > 0:
        odom_topic = sys.argv[0]
    else:
        odom_topic = Config.odom_topic

    # launch node
    rospy.init_node('show_angle')
    r = rospy.Rate(10)
    sub = rospy.Subscriber (Config.odom_topic, Odometry, get_rotation)

    # print current robot's angle
    while not rospy.is_shutdown():
        yaw_deg = yaw*180/math.pi
        rospy.loginfo("current angle: %f", yaw_deg)
        r.sleep()
