#!/usr/bin/env python3
import socket
import struct
import time
import select
from multiprocessing import Process, Value, SimpleQueue


class UDPNode(Process):
    """
    UDPNode
    ----------
    A multiprocessing UDP communication helper.

    Example:
        node = UDPNode(port=8889, host_ip="192.168.0.10")
        node.set_callback(my_callback)
        node.start()
    """

    def __init__(self, port: int, host_ip: str):
        super().__init__()
        self.port = port
        self.host_ip = host_ip
        self.callback = None
        self._stop_flag = Value('b', 0)  # binary flag
        self.message_queue = SimpleQueue()
        self.sock = None  # socket created later in run()

    def set_callback(self, func):
        """Register a callback to be called on incoming UDP messages."""
        self.callback = func

    def run(self):
        """Main loop, executed when Process.start() is called."""
        self._stop_flag.value = 0

        # Create socket here (multiprocessing-safe)
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.host_ip, self.port))
            print(f"[UDPNode] Listening on {self.host_ip}:{self.port}")
        except socket.error as e:
            print(f"[UDPNode] Socket error: {e}")
            return

        while self._stop_flag.value == 0:
            try:
                read_list, _, _ = select.select([self.sock], [], [], 0.1)
                if self.sock in read_list:
                    data = self.sock.recvfrom(1024)
                    if self.callback:
                        try:
                            self.callback(data)
                        except Exception as e:
                            print(f"[UDPNode] Callback error: {e}")
            except Exception as e:
                print(f"[UDPNode] Loop error: {e}")

            # Handle outgoing queue
            while not self.message_queue.empty():
                msg, addr = self.message_queue.get()
                try:
                    self.sock.sendto(msg, addr)
                except socket.error as e:
                    print(f"[UDPNode] Failed to send message: {e}")

        self.sock.close()
        print("[UDPNode] Socket closed")

    def sendto(self, msg: bytes, ip: str, port: int):
        """Queue a message for sending."""
        self.message_queue.put((msg, (ip, port)))

    def stop(self):
        """Stop the UDP process."""
        self._stop_flag.value = 1


# Utility functions for packing/unpacking structured messages
def render_pack(dictionary):
    try:
        return struct.pack('=hhhihidd',
                           dictionary['Header'],
                           dictionary['ForceSensor'],
                           dictionary['InteractionForce'],
                           dictionary['RobotVelocity'],
                           dictionary['RobotPosition'],
                           dictionary['TargetPosition'],
                           dictionary['TaskData'],
                           dictionary['AdditionalData'])
    except struct.error as e:
        print(f"[UDPNode] Packing error: {e}")
        raise


def render_unpack(msg: bytes):
    try:
        fields = struct.unpack('=hhhihidd', msg)
        return {
            'Header': fields[0],
            'ForceSensor': fields[1],
            'InteractionForce': fields[2],
            'RobotVelocity': fields[3],
            'RobotPosition': fields[4],
            'TargetPosition': fields[5],
            'TaskData': fields[6],
            'AdditionalData': fields[7]
        }
    except struct.error as e:
        print(f"[UDPNode] Unpacking error: {e}")
        raise


# Example usage (standalone test, not ROS-dependent)
if __name__ == "__main__":
    def debug_callback(md):
        msg, addr = md
        print(f"[UDPNode] Received {len(msg)} bytes from {addr}")
        try:
            val = render_unpack(msg)
            print("Decoded message:", val)
        except Exception as e:
            print(f"Decode error: {e}")

    node = UDPNode(8889, "127.0.0.1")
    node.set_callback(debug_callback)
    node.start()

    test_dict = {
        'Header': 1,
        'ForceSensor': 2,
        'InteractionForce': 3,
        'RobotVelocity': 4,
        'RobotPosition': 5,
        'TargetPosition': 6,
        'TaskData': 7,
        'AdditionalData': 8
    }

    time.sleep(1.0)
    node.sendto(render_pack(test_dict), "127.0.0.1", 8889)

    input("Press Enter to stop...")
    node.stop()
    node.join()
