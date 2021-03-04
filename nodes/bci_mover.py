#!/usr/bin/env python
from config import Config
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import socket
from move_forward import move_forward
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

if __name__ == "__main__":
    # init global vars
    Config.pose = dict({'x':0, 'y':0, 'yaw':0})
    Config.command =Twist()
    Config.clockwise = -1
    Config.anticlockwise = +1
    Config.turning = False
    Config.turning_direction = 0
    Config.old_diff = None
    Config.port = rospy.get_param('~socket_port')
    Config.ip = rospy.get_param('~socket_ip')

    # init node & topics
    rospy.init_node("bci_mover")
    rospy.loginfo("initializing `bci_mover` node")
    publisher = rospy.Publisher(Config.cmd_vel_topic, Twist, queue_size=1)
    rospy.Subscriber (Config.odom_topic, Odometry, odom_callback)
    rospy.Subscriber(Config.laser_topic, LaserScan, laser_callback)
    rate = rospy.Rate(10)

    # init socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('', Config.port))
    s.listen(Config.queue)

    # main loop
    rospy.loginfo("~ Waiting for client...")
    newSocket, address = s.accept()
    rospy.loginfo("  New request from %s", address)
    old_speed = 0
    socket_listening = True
    try:
        while socket_listening:
            status = 1
            emergency_stop = False
            rospy.loginfo("    1. Waiting for command")
            #data, addr = serverSocket.recvfrom(bufferSize)
            data = newSocket.recv(Config.bufferSize)
            with open("../logs/socket-log.txt", "a") as f:
                f.write("\n" + str(data))
            if len(data) < 2 or not(data[0].isdigit()) or not(data[1].isdigit()):
                rospy.loginfo("    2. Command `%s` not supported.", data)
                status = 0
            elif data == "00":
                rospy.loginfo("    2. Command `00` recieved. Closing socket.")
                status = 0
                socket_listening = False
            else:
                rospy.loginfo("    2. Command recieved `%s`.", data)
                direction = int(data[0])
                speed = int(data[1])
                if speed == 0:
                    speed = old_speed
                else:
                    old_speed = speed
                rospy.loginfo("    Executing move: direction=`%d` speed=`d`".format(direction, speed))
                turn_to_angle(direction, speed,  publisher, rate)
                if Config.move_forward_at_each_move:
                    move_forward(speed, publisher, rate)
            rospy.loginfo("    3. Sending back robot status...")
            newSocket.send(str(status))
        newSocket.close()
    finally:
        s.close()
