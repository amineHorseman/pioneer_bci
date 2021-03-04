#!/usr/bin/env python
from config import Config
import math
import rospy

def move_forward(speed, publisher, rate):
    Config.command.linear.x = speed * Config.speed_reduce_factor
    rospy.logdebug("moving forward with speed {}...".format(Config.command.linear.x)) 
    x_start = Config.x
    y_start = Config.y
    distance = 0
    while not rospy.is_shutdown() and distance < Config.move_distance:
        rospy.logdebug("x=%f y=%f (travelled=%f)", x,y,distance)
        if not(math.isnan(Config.obstacle_distance)):
            rospy.logdebug("obstacle distance = %f", Config.obstacle_distance)
        if Config.obstacle_distance < Config.obstacle_min_distance: 
            # emergency_stop:
            rospy.logdebug("stopping [obstacle ahead]")
            break
        else:
            # go forward
            if distance + Config.slow_down_distance > Config.move_distance:
                Config.command.linear.x = 0.1
                rospy.logdebug("slowing down to speed 0.1")
            publisher.publish(Config.command)
            rate.sleep()
            distance = math.sqrt((Config.x-x_start)*(Config.x-x_start) + 
                                 (Config.y-y_start)*(Config.y-y_start))

    rospy.loginfo("Stopped. Travelled disance = %f", distance)
    stop_motors(Config.command, publisher)

def stop_motors(command, publisher): 
    command.angular.z = 0
    command.linear.x = 0
    publisher.publish(command)
