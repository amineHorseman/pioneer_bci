#!/usr/bin/env python
from config import Config
from geometry_msgs.msg import Twist
from move_forward import move_forward
from nav_msgs.msg import Odometry
import os
import rospy
from sensor_msgs.msg import LaserScan
import socket
from std_msgs.msg import Float64, Bool
from tf.transformations import euler_from_quaternion
from turn import turn_to_angle, parse_angle

def laser_callback(scan):
    min_angle, max_angle = Config.laserscan_range
    Config.obstacle_distance = min(scan.ranges[min_angle:max_angle])

def odom_callback(msg):
    orientation = msg.pose.pose.orientation
    orientation_coord = [orientation.x, orientation.y, orientation.z, orientation.w]
    (_, _, Config.pose["yaw"]) = euler_from_quaternion(orientation_coord)
    pose = msg.pose.pose.position
    (Config.pose["x"], Config.pose["y"]) = (pose.x, pose.y)

def get_param(name, default_value):
    value = rospy.get_param(name, default="")
    if value == "":
        value = default_value
    print("param: ", name, ', ', value)
    return value

if __name__ == "__main__":
    # init global vars
    Config.pose = dict({'x':0, 'y':0, 'yaw':0})
    Config.clockwise = -1
    Config.anticlockwise = +1
    Config.turning = False
    Config.turning_direction = 0
    Config.old_angle_diff = None
    ip = get_param("ip", Config.ip)
    port = get_param("port", Config.port)
    cmd_vel_topic = get_param("cmd_vel_topic", Config.cmd_vel_topic)
    laser_topic = get_param("laser_topic", Config.laser_topic)
    odom_topic = get_param("odom_topic", Config.odom_topic)
    if not os.path.isdir("../logs"):
        os.mkdir("../logs")

    # init node & topics
    rospy.init_node("bci_mover")
    rospy.loginfo("initializing `bci_mover` node")
    publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
    rospy.Subscriber (odom_topic, Odometry, odom_callback)
    rospy.Subscriber(laser_topic, LaserScan, laser_callback)
    rospy.loginfo("Listening to topics: %s, %s. Publishing to %s", 
        odom_topic, laser_topic, cmd_vel_topic)
    rate = rospy.Rate(10)

    # init socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('', Config.port))
    s.listen(Config.queue)

    # main loop
    restart = 1
    while True:
        rospy.loginfo("~ Waiting for client...")
        newSocket, address = s.accept()
        rospy.loginfo("  New request from %s", address)
        old_speed = 0
        socket_listening = True
        try:
            while socket_listening:
                status = '1'
                emergency_stop = False
                rospy.loginfo("    1. Waiting for command...")
                data = newSocket.recv(Config.bufferSize).decode()
                with open("../logs/socket-log.txt", "a") as f:
                    f.write("\n" + str(data))
                if len(data) < 2 or not(data[0].isdigit()) or not(data[1].isdigit()):
                    rospy.loginfo("    2. Command `%s` not supported", data)
                    status = '0'
                elif data == "00":
                    rospy.loginfo("    2. Command `00` recieved. Closing socket...")
                    status = '0'
                    socket_listening = False
                else:
                    rospy.loginfo("    2. Command recieved `%s`.", data)
                    direction = int(data[0])
                    speed = int(data[1])
                    if speed == 0:
                        speed = old_speed
                    else:
                        old_speed = speed
                    rospy.loginfo("    Executing move: direction=`%d` speed=`%d`", direction, speed)
                    if direction == 8 and Config.move_backward:
                    	move_forward(-speed, publisher, rate)
                    else:
                    	turn_to_angle(direction, speed,  publisher, rate)
                    	if Config.move_forward_at_each_move or direction == 2:
                    		move_forward(speed, publisher, rate)
                    	else:
                        	publisher.publish(Twist())
                rospy.loginfo("    3. Sending back robot status...")
                newSocket.send(status.encode())
            newSocket.close()
        except socket.timeout:
            rospy.loginfo("    Socket timout")
            newSocket.close()
            restart += 1
            if restart > 3:
                rospy.loginfo("    Registred 3 socket timeout. Shutting down node")
                break
        finally:
            if restart > 3:
                s.close()
