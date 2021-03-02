#!/usr/bin/env python

from __future__ import print_function
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import socket

print('turn to angle') 
#### config #### 
angle_error = 0.1
move_distance = 1
obstacle_min_distance = 0.6
port = 12347
#ip = '10.3.66.123'
ip = 'localhost'
# cmd_vel_topic = 'cmd_vel_mux/input/teleop'
cmd_vel_topic = 'cmd_vel'
laser_topic = 'scan'
#odom_topic = 'odom'
odom_topic = 'pose'
regulated_speeds = {0.03: 0,
                    0.1: 0.01,
                    1: 0.03,
                    3: 0.1,
                    20: 0.3,
                    45: 0.5,
                    360: 0.5}  # mapping angles with angular speeds
speed_reduce_factor = 0.1  # reducing linear speed
slow_down_distance = 0.2  # reduce speed to minimum (0.1) under this distance from target (in meters)
#### config end #### 

(x, y) = (0, 0)
yaw = 0.0
obstacle_distance = 0
clockwise = -1
anticlockwise = +1
turning = False
turning_direction = 0
old_diff = None

def laser_callback(scan):
    global obstacle_distance
    obstacle_distance = min(scan.ranges[160:480])

def get_rotation(msg):
    global yaw, x, y
    orientation = msg.pose.pose.orientation
    orientation_coord = [orientation.x, orientation.y, orientation.z, orientation.w]
    (_, _, yaw) = euler_from_quaternion(orientation_coord)
    pose = msg.pose.pose.position
    # (x, y, z) = (pose.x, pose.y, pose.z)
    (x, y) = (pose.x, pose.y)

def regulated_angular_speed(angle_diff):
    angle_diff = abs(angle_diff)
    proportional_speed = 0
    for k, v in sorted(regulated_speeds.items()):
    	if (k < angle_diff):
            proportional_speed = v
    print("rotating at speed={} (angle_diff={})".format(proportional_speed, angle_diff))
    return proportional_speed

def sign(value):
    return -1 if value < 0 else 1

def get_rotation_direction(current, target):
    if sign(current) == sign(target):
        return (anticlockwise if target > current else clockwise)
    else:
        a = abs(current) + abs(target)
        return (anticlockwise if a < 360-a else clockwise)*sign(target)

def get_closest_rotation_angle(old_angle, target_angle):
    angle_diff_left = (old_angle - target_angle) % 360
    angle_diff_right = (target_angle - old_angle) % 360
    turning_angle = -angle_diff_left if angle_diff_left < angle_diff_right \
                                     else angle_diff_right
    return turning_angle

def stop_motors(): 
    command.angular.z = 0
    command.linear.x = 0
    pub.publish(command)


rospy.init_node('rotate_robot')

print('init') 
pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
rospy.Subscriber (odom_topic, Odometry, get_rotation)
rospy.Subscriber(laser_topic, LaserScan, laser_callback)
r = rospy.Rate(10)
command =Twist()

def drive(target, speed):
    global turning, turning_direction, old_diff
    print("rotating to {}...".format(target))
    while not rospy.is_shutdown() and target is not None:
        yaw_deg = yaw*180/math.pi
        #diff_deg = target-yaw_deg
        diff_deg = get_closest_rotation_angle(yaw_deg, target)
        print("  target={} current:{} diff:{}".format(target, yaw_deg, diff_deg))
        if (abs(diff_deg) < angle_error):
            print("final angle_error = ", diff_deg)
            break
        if old_diff is not None and sign(old_diff) != sign(diff_deg) and abs(diff_deg) < 3:
            # avoid p3dx angle diff switching sign
            print("angle diff switch sign. Stop truning")
            break
        if not turning:
            # get rotation direction and save it to keep it for next iteration
            # (avoids oscillations)
            turning_direction = get_rotation_direction(yaw_deg, target)
            turning = True
            old_diff = diff_deg
        command.angular.z = regulated_angular_speed(diff_deg) * turning_direction
        pub.publish(command)
        r.sleep()
        old_diff = diff_deg

    stop_motors()
    turning = False
    old_diff = None

    command.linear.x = speed * speed_reduce_factor
    print("moving forward with speed {}...".format(command.linear.x)) 
    x_start = x
    y_start = y
    distance = 0
    while not rospy.is_shutdown() and distance < move_distance:
        print("x={} y={} (travelled={})".format(x,y,distance))
        if not(math.isnan(obstacle_distance)):
            print("obstacle distance = ",obstacle_distance)
        if obstacle_distance < obstacle_min_distance: 
            # emergency_stop:
            print("stopping [obstacle ahead]")
            break
        else:
            # go forward
            if distance + slow_down_distance > move_distance:
                command.linear.x = 0.1
                print("slowing down to speed 0.1")
            pub.publish(command)
            r.sleep()
            distance = math.sqrt((x-x_start)*(x-x_start) + (y-y_start)*(y-y_start))

    print ('Stopped. Travelled disance=', distance)
    stop_motors()

"""
def parse_angle(direction):  # for 3rd person mode
    angle = [None, 45, 0, -45, 90, None, -90, 135, 180, -135] # 0 and 5 => do nothing
    return angle[direction]
"""

def parse_angle(direction):  # for 1st person mode
    angle = [0, 135, 180, -135, 90, 0, -90, 45, 0, -45] # 0 and 5 => do nothing
    offset = angle[direction]
    yaw_deg = yaw*180/math.pi
    new = yaw_deg + offset
    print("#######  offset = ", offset, " - old = ", yaw_deg, "new = ", new)
    return new

bufferSize = 256
queue = 5
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('', port))
s.listen(queue)


print("~ Waiting for client...")
newSocket, address = s.accept()
print("  New request from {}".format(address))
old_speed = 0
try:
    while True:
        status = 1
        emergency_stop = False
        print("    1. Waiting for command")
        #data, addr = serverSocket.recvfrom(bufferSize)
        data = newSocket.recv(bufferSize)
        with open("../logs/socket-log.txt", "a") as f:
            f.write("\n" + str(data))
        if not(data[0].isdigit()) or not(data[1].isdigit()):
            print("    2. Commands recieved `{}` are not digits", data)
            print("    3. Ignoring command")
            status = 0
        elif data[0] == '0':
            print("    2. Command recieved: direction `0` does not exist")
            print("    3. Ignoring command")
            status = 0
        else:
            direction = int(data[0])
            angle = parse_angle(direction)
            speed = int(data[1])
            speed = int(speed)
            if speed == 0:
                speed = old_speed
            else:
                old_speed = speed
            print("    2. Command recieved: direction={} speed={}".format(direction, speed))
            print("    3. Moving the robot...")
            drive(angle, speed)
        print("    4. Sending last move status...")
        newSocket.send(str(status))
finally:
    s.close()
