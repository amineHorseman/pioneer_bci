#!/usr/bin/env python
from config import Config

import socket
port = Config.port
ip = Config.ip

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.connect((ip, port))

while True:
        data = input('enter direction and speed: ')
        s.sendall(str(data).encode())
        _ = s.recv(256)
        if data == '00':
            s.close
            break
