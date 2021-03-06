#!/usr/bin/env python
from config import Config
from geometry_msgs.msg import Twist
import rospy
import math

def turn_to_angle(direction, speed, publisher, rate):
    command = Twist()
    target_angle = parse_angle(direction)
    rospy.logdebug("rotating to %f...", target_angle)
    while not rospy.is_shutdown() and target_angle is not None:
        current_angle = Config.pose["yaw"]*180/math.pi
        angle_diff = get_closest_rotation_angle(current_angle, target_angle)
        rospy.logdebug("  target_angle: %f - current: %f - diff: %f", target_angle, current_angle, angle_diff)
        if (abs(angle_diff) < Config.angle_error):
            rospy.logdebug("final angle_error = %f", angle_diff)
            break
        if Config.old_angle_diff is not None and sign(Config.old_angle_diff) != sign(angle_diff) and abs(angle_diff) < 3:
            # handle the case when odom angle stepping from 179 to -179 while we're trying to reach 180
            rospy.logdebug("angle diff switch sign. Stop truning")
            break
        if not Config.turning:
            # get rotation direction and save it to keep it for next iteration (avoids oscillations)
            Config.turning_direction = get_rotation_direction(current_angle, target_angle)
            Config.turning = True
            Config.old_angle_diff = angle_diff
        command.angular.z = regulated_angular_speed(angle_diff) * Config.turning_direction
        publisher.publish(command)
        rate.sleep()
        Config.old_angle_diff = angle_diff
        
    # stop motors
    publisher.publish(Twist())
    Config.turning = False

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

def parse_angle(direction):
    angle = [0, 45, 0, -45, 90, 0, -90, 135, 180, -135] # 0 and 5 => do nothing
    offset = angle[direction]
    if not Config.first_person_mode:
        # use relative angles
        return offset
    else:
        # use absolute angles
        old_angle = Config.pose["yaw"]*180/math.pi
        new_angle = old_angle + offset
        rospy.logdebug("  offset = %f, - old_ang = %f - new_ang = %f", offset, old_angle, new_angle)
        return new_angle
