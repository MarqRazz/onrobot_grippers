#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import onrobot_rg_modbus_tcp.comModbusTcp
from onrobot_rg_control.baseOnRobotRG import OnRobotBaseRG
from onrobot_rg_msgs.msg import OnRobotRGInput
from onrobot_rg_msgs.msg import OnRobotRGOutput
from std_srvs.srv import Trigger

class OnRobotRGTcpNode(OnRobotBaseRG):

    def __init__(self):
        # init the node and fetch some params
        super().__init__('OnRobotRGTcpNode')

        self.ip = self.declare_parameter('/onrobot/ip', '192.168.1.1').get_parameter_value().string_value
        self.port = self.declare_parameter('/onrobot/port', '502').get_parameter_value().string_value
        self.changer_addr = self.declare_parameter('/onrobot/changer_addr', 65).get_parameter_value().integer_value
        self.dummy = self.declare_parameter('/onrobot/dummy', False).get_parameter_value().bool_value
        # Gripper is a RG gripper with a Modbus/TCP connection
        self.client = onrobot_rg_modbus_tcp.comModbusTcp.communication(self.dummy)
        self.prev_msg = []

        # the communication lib needs access to the node's logger
        self.client.logger = self.get_logger()

        # Connects to the ip address received as an argument
        if not self.client.connectToDevice(self.ip, self.port, self.changer_addr):
            self.get_logger().error("Could not connect to device")
            # XXX Just crash out here


        # The Gripper status is published on the topic named 'OnRobotRGInput'
        self.pub = self.create_publisher(OnRobotRGInput, 'OnRobotRGInput', 1)

        # The restarting service
        self.rst_srv = self.create_service(
            Trigger,
            "/onrobot_rg/restart_power",
            self.restartPowerCycle)

        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # The Gripper command is received from the topic named 'OnRobotRGOutput'
        self.sub = self.create_subscription(
            OnRobotRGOutput,
            'OnRobotRGOutput',
            self.refresh_command,
            10)


    def restartPowerCycle(self, request, response):
        self.get_logger().info("Restarting the power cycle of all grippers connected.")
        # Called on the client directly: this method shadows the one the base
        # class provides, so going through self would recurse into the service.
        response.success = bool(self.client.restartPowerCycle())
        response.message = ("Power cycle restarted." if response.success
                            else "Restarting the power cycle failed.")
        return response


    def timer_callback(self):
        # Nothing may escape this callback: an exception here stops the timer
        # and takes the node down with it.
        try:
            status = self.getStatus()
            if status is None:
                # Publish nothing rather than a made up status: consumers of
                # OnRobotRGInput are expected to fail closed when it goes stale.
                return
            self.pub.publish(status)

            # Send the most recent command
            if not int(format(status.g_sta, '016b')[-1]):  # not busy
                if not self.prev_msg == self.message:       # find new message
                    self.get_logger().info(self.get_name()+": Sending message.")
                    if self.sendCommand():
                        # Only advance once the write has actually reached the
                        # device, so that a failed write is retried on the next
                        # tick instead of being dropped.
                        self.prev_msg = list(self.message)
        except Exception as e:
            self.get_logger().error(
                f"Unhandled error in the timer callback: {e}",
                throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = OnRobotRGTcpNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
