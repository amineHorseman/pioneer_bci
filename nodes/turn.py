#!/usr/bin/env python
from config import Config
import rospy
import math

def turn_to_angle(direction, speed, publisher, rate):
    target = parse_angle(direction)
    rospy.logdebug("rotating to %f...", target)
    while not rospy.is_shutdown() and target is not None:
        yaw_deg = Config.pose["yaw"]*180/math.pi
        #diff_deg = target-yaw_deg
        diff_deg = get_closest_rotation_angle(yaw_deg, target)
        rospy.logdebug("  target=%f - current:%f - diff:%f", target, yaw_deg, diff_deg)
        if (abs(diff_deg) < Config.angle_error):
            rospy.logdebug("final angle_error = %f", diff_deg)
            break
        if Config.old_diff is not None and sign(Config.old_diff) != sign(diff_deg) and abs(diff_deg) < 3:
            # avoid p3dx angle diff switching sign
            rospy.logdebug("angle diff switch sign. Stop truning")
            break
        if not Config.turning:
            # get rotation direction and save it to keep it for next iteration
            # (avoids oscillations)
            Config.turning_direction = get_rotation_direction(yaw_deg, target)
            Config.turning = True
            Config.old_diff = diff_deg
        Config.command.angular.z = regulated_angular_speed(diff_deg) * Config.turning_direction
        publisher.publish(Config.command)
        rate.sleep()
        Config.old_diff = diff_deg

def get_closest_rotation_angle(old_angle, target_angle):
    angle_diff_left = (old_angle - target_angle) % 360
    angle_diff_right = (target_angle - old_angle) % 360
    turning_angle = -angle_diff_left if angle_diff_left < angle_diff_right \
                                     else angle_diff_right
    return turning_angle

def sign(value):
    return -1 if value < 0 else 1

def get_rotation_direction(current, target):
    if sign(current) == sign(target):
        return (Config.anticlockwise if target > current else Config.clockwise)
    else:
        a = abs(current) + abs(target)
        return (Config.anticlockwise if a < 360-a else Config.clockwise)*sign(target)

def regulated_angular_speed(angle_diff):
    angle_diff = abs(angle_diff)
    proportional_speed = 0
    for k, v in sorted(Config.regulated_speeds.items()):
    	if (k < angle_diff):
            proportional_speed = v
    rospy.logdebug("rotating at speed=%f (angle_diff=%f)", proportional_speed, angle_diff)
    return proportional_speed

# TODO: integrate 2 'parse_angle' fct & set mode as Config var 
"""
def parse_angle(direction):  # for 3rd person mode
    angle = [None, 45, 0, -45, 90, None, -90, 135, 180, -135] # 0 and 5 => do nothing
    return angle[direction]
"""

def parse_angle(direction):  # for 1st person mode
    angle = [0, 135, 180, -135, 90, 0, -90, 45, 0, -45] # 0 and 5 => do nothing
    offset = angle[direction]
    yaw_deg = Config.pose["yaw"]*180/math.pi
    new = yaw_deg + offset
    rospy.logdebug("  offset = %f, - old_ang = %f - new_ang = %f", offset, yaw_deg, new)
    return new
