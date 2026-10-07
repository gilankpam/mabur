#!/usr/bin/env python3
import socket,sys
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.settimeout(2)
for cmd in sys.argv[1:]:
    s.sendto(cmd.encode(),("127.0.0.1",8303))
    try: print(s.recv(2048).decode().strip())
    except socket.timeout: print("(no reply)", cmd)
