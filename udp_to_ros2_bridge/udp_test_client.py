#!/usr/bin/env python3
import socket
import time
import random

UDP_IP = "127.0.0.1"    # The IP your node listens on
UDP_PORT = 61559         # The UDP port your node listens on

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

counter = 0
try:
    while True:
        counter += 1
        # Create a test message with changing values
        msg = f"Message {counter} | value1={random.randint(0,100)} | value2={random.random():.2f}"
        sock.sendto(msg.encode('utf-8'), (UDP_IP, UDP_PORT))
        print(f"Sent: {msg}")
        time.sleep(1)  # Send one message per second
except KeyboardInterrupt:
    print("Stopped sending messages")
