#!/usr/bin/env python3
"""
Module comModbusTcp: defines a class which communicates with
OnRobot Grippers using the Modbus/TCP protocol.
"""

import sys
import rclpy
import threading
from pymodbus.client import ModbusTcpClient
#from pymodbus.client import ModbusSerialClient

class communication:

    def __init__(self, dummy=False):
        self.client = None
        self.dummy = dummy
        self.lock = threading.Lock()
        self.logger = None

    def connectToDevice(self, ip: str, port: str, changer_addr: int = 65) -> bool:
        """Connects to the client.
           The method takes the IP address and port number
           (as a string, e.g. '192.168.1.1' and '502') as arguments.
        """
        if self.dummy:
            if self.logger:
                self.logger.info(
                    sys._getframe().f_code.co_name)
            return True

        self.client = ModbusTcpClient(
            host=ip,
            port=port,
            stopbits=1,
            bytesize=8,
            parity='E',
            baudrate=115200,
            timeout=1)
        self.changer_addr = changer_addr
        return self.client.connect()

    def disconnectFromDevice(self):
        """Closes connection."""
        if self.dummy:
            if self.logger:
                self.logger.info(
                    sys._getframe().f_code.co_name)
            return

        self.client.close()

    def logWarning(self, text):
        """Logs a warning if the node handed its logger over."""
        if self.logger:
            self.logger.warn(text, throttle_duration_sec=5.0)

    def transactionFailed(self, result):
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
            self.logWarning(f"reconnect failed: {e}")

    def sendCommand(self, message):
        """Sends a command to the Gripper.
           The method takes a list of uint8 as an argument.
           Returns True if the command reached the device.
        """
        if self.dummy:
            if self.logger:
                self.logger.info(
                    sys._getframe().f_code.co_name)
            return True

        # Send a command to the device (address 0 ~ 2)
        if message != []:
            with self.lock:
                try:
                    result = self.client.write_registers(
                        address=0, values=message, slave=self.changer_addr)
                except Exception as e:
                    result = e
                if self.transactionFailed(result):
                    self.logWarning(f"writing the command failed: {result}")
                    self.reconnect()
                    return False
        return True

    def restartPowerCycle(self):
        """Restarts the power cycle of Compute Box
           Necessary is Safety Switch of the grippers are pressed
           Writing 2 to this field powers the tool off for a short amount of time and then powers them back
        """
        message = 2
        restart_address = 63

        # Sending 2 to address 0x0 resets compute box (address 63) power cycle
        with self.lock:
            try:
                result = self.client.write_registers(
                    address=0, values=[message], slave=restart_address)
            except Exception as e:
                result = e
            if self.transactionFailed(result):
                self.logWarning(f"restarting the power cycle failed: {result}")
                self.reconnect()
                return False
        return True

    def getStatus(self):
        """Sends a request to read, wait for the response
           and returns the Gripper status.
           The method gets by specifying register address as an argument.
           Returns None if the status could not be read.
        """
        response = [0] * 18
        if self.dummy:
            if self.logger:
                self.logger.info(
                    sys._getframe().f_code.co_name)
            return response

        # Get status from the device (address 258 ~ 275)
        with self.lock:
            try:
                response = self.client.read_holding_registers(
                    address=258, count=18, slave=self.changer_addr)
            except Exception as e:
                response = e
            if self.transactionFailed(response) or not hasattr(response, 'registers'):
                self.logWarning(f"reading the status failed: {response}")
                self.reconnect()
                return None

        # Output the result
        return response.registers
