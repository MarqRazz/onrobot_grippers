#!/usr/bin/env python3
"""
Module comModbusTcp: defines a class which communicates with
OnRobot Grippers using the Modbus/TCP protocol.
"""

import sys
import threading
from pymodbus.client import ModbusTcpClient
import rclpy
from rclpy.node import Node


class Communication(Node):

    def __init__(self, dummy=False):
        super().__init__('communication_node')
        self.client = None
        self.dummy = dummy
        self.lock = threading.Lock()

    def connect_to_device(self, ip, port, changer_addr=65):
        """Connects to the client.
           The method takes the IP address and port number
           (as a string, e.g. '192.168.1.1' and '502') as arguments.
        """
        if self.dummy:
            self.get_logger().info(f"{self.get_name()}: {sys._getframe().f_code.co_name}")
            return
        self.client = ModbusTcpClient(
            ip,
            port=port,
            stopbits=1,
            bytesize=8,
            parity='E',
            baudrate=115200,
            timeout=1)
        self.changer_addr = changer_addr
        self.client.connect()

    def disconnect_from_device(self):
        """Closes connection."""
        if self.dummy:
            self.get_logger().info(f"{self.get_name()}: {sys._getframe().f_code.co_name}")
            return
        self.client.close()

    def send_command(self, message):
        """Sends a command to the Gripper.
           The method takes a list of uint8 as an argument.
        """
        if self.dummy:
            self.get_logger().info(f"{self.get_name()}: {sys._getframe().f_code.co_name}")
            return
        # Send a command to the device (address 0 ~ 1)
        if message:
            command = [message[0] + message[1],
                       message[2] + message[3]]
            with self.lock:
                self.client.write_registers(
                    address=0, values=command, slave=int(self.changer_addr))

    def get_status(self):
        """Sends a request to read, wait for the response
           and returns the Gripper status.
           The method gets by specifying register address as an argument.
        """
        response = [0] * 2
        if self.dummy:
            self.get_logger().info(f"{self.get_name()}: {sys._getframe().f_code.co_name}")
            return response

        # Get status from the device (address 258 ~ 259)
        with self.lock:
            response = self.client.read_holding_registers(
                address=258, count=2, slave=int(self.changer_addr)).registers

        # Output the result
        return response


def main(args=None):
    rclpy.init(args=args)
    communication_node = Communication(dummy=False)

    try:
        rclpy.spin(communication_node)
    except KeyboardInterrupt:
        pass
    finally:
        communication_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()