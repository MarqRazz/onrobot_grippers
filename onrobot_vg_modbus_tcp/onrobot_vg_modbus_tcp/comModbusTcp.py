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

    def transaction_failed(self, result):
        """Tells whether a Modbus transaction failed.
           pymodbus returns a ModbusIOException instead of raising it when a
           transaction times out or the socket breaks, so the returned object
           has to be inspected before its payload is used. Exception instances
           and error responses are both treated as a failure, which keeps this
           correct whether or not the installed version provides isError().
        """
        if result is None or isinstance(result, Exception):
            return True
        is_error = getattr(result, 'isError', None)
        return bool(is_error()) if callable(is_error) else False

    def reconnect(self):
        """Re-establishes the connection after a failed transaction so that
           the node recovers on its own once the link comes back.
        """
        try:
            self.client.close()
            self.client.connect()
        except Exception as e:
            self.get_logger().warn(
                f"{self.get_name()}: reconnect failed: {e}",
                throttle_duration_sec=5.0)

    def send_command(self, message):
        """Sends a command to the Gripper.
           The method takes a list of uint8 as an argument.
           Returns True if the command reached the device.
        """
        if self.dummy:
            self.get_logger().info(f"{self.get_name()}: {sys._getframe().f_code.co_name}")
            return True
        # Send a command to the device (address 0 ~ 1)
        if message:
            command = [message[0] + message[1],
                       message[2] + message[3]]
            with self.lock:
                try:
                    result = self.client.write_registers(
                        address=0, values=command, slave=int(self.changer_addr))
                except Exception as e:
                    result = e
                if self.transaction_failed(result):
                    self.get_logger().warn(
                        f"{self.get_name()}: writing the command failed: {result}",
                        throttle_duration_sec=5.0)
                    self.reconnect()
                    return False
        return True

    def get_status(self):
        """Sends a request to read, wait for the response
           and returns the Gripper status.
           The method gets by specifying register address as an argument.
           Returns None if the status could not be read.
        """
        response = [0] * 2
        if self.dummy:
            self.get_logger().info(f"{self.get_name()}: {sys._getframe().f_code.co_name}")
            return response

        # Get status from the device (address 258 ~ 259)
        with self.lock:
            try:
                response = self.client.read_holding_registers(
                    address=258, count=2, slave=int(self.changer_addr))
            except Exception as e:
                response = e
            if self.transaction_failed(response) or not hasattr(response, 'registers'):
                self.get_logger().warn(
                    f"{self.get_name()}: reading the status failed: {response}",
                    throttle_duration_sec=5.0)
                self.reconnect()
                return None

        # Output the result
        return response.registers


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