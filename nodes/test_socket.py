#!/usr/bin/env python


import socket
port = 12347
ip = '127.0.0.1'

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.connect((ip, port))

while True:
        data = input('enter direction and speed: ')
        s.sendall(str(data))
        data = s.recv(256)
