#!/usr/bin/env python
from config import Config
from geometry_msgs.msg import Twist
import math
import rospy

def move_forward(speed, publisher, rate):
    command = Twist()
    command.linear.x = speed * Config.speed_reduce_factor
    rospy.logdebug("moving forward with speed %f...", command.linear.x)
    x_start = Config.pose["x"]
    y_start = Config.pose["y"]
    distance = 0
    while not rospy.is_shutdown() and distance < Config.move_distance:
        rospy.logdebug("x=%f y=%f (travelled=%f)", Config.pose["x"],Config.pose["y"],distance)
        if not(math.isnan(Config.obstacle_distance)):
            rospy.logdebug("obstacle distance = %f", Config.obstacle_distance)
        if Config.obstacle_distance < Config.obstacle_min_distance: 
            # emergency_stop:
            rospy.logdebug("stopping [obstacle ahead]")
            break
        else:
            # go forward
            if distance + Config.slow_down_distance > Config.move_distance:
                command.linear.x = 0.1
                rospy.logdebug("slowing down to speed 0.1")
            publisher.publish(command)
            rate.sleep()
            distance = math.sqrt((Config.pose["x"]-x_start)*(Config.pose["x"]-x_start) + 
                                 (Config.pose["y"]-y_start)*(Config.pose["y"]-y_start))

    # stop motors
    publisher.publish(Twist())
    rospy.loginfo("Stopped. Travelled disance = %f", distance)

    return distance
