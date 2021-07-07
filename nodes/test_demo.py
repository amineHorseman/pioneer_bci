#!/usr/bin/env python

from __future__ import print_function
import math
import socket
 
#### config #### 
angle_error = 0.1
move_distance = 2
obstacle_min_distance = 2
port = 12347
#ip = '10.3.66.5'
#ip = '10.3.67.255'
# ip = '10.3.66.123'
ip = '192.168.43.78'

bufferSize = 256
queue = 5
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('', port))
s.listen(queue)


print("~ Waiting for client...")
newSocket, address = s.accept()
print("  New request from {}".format(address))
ready = '1'
old_speed = 0
trials_count = 1
try:
    while 1: 
        print("    1. Waiting for command")
        data = newSocket.recv(bufferSize)
        print("    2. Trial {} Command recieved: direction={}".format(trials_count, data.decode()))
        print("    3. Sending `ready` status...")
        print("****")
        newSocket.send(ready.encode())
        trials_count += 1
finally:
    s.close()
