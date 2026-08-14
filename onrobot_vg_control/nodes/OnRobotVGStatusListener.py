#!/usr/bin/env python3

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from onrobot_vg_msgs.msg import OnRobotVGInput


class OnRobotVGStatusListener(Node):
    def __init__(self):
        super().__init__('onrobot_vg_status_listener')
        self.subscription = self.create_subscription(
            OnRobotVGInput,
            'OnRobotVGInput',
            self.print_status_callback,
            10)
        self.subscription  # prevent unused variable warning

    def print_status_callback(self, msg):
        """Callback function to handle the incoming OnRobotVGInput message."""
        logger = self.get_logger()
        # Skip building the string at all unless DEBUG is actually enabled.
        if logger.get_effective_level() > LoggingSeverity.DEBUG:
            return
        logger.debug(self.status_interpreter(msg))

    def status_interpreter(self, status):
        """Generates a string according to the current value of the status variables."""
        output = '\n-----\nOnRobot VG status interpreter\n-----\n'

        # g_vca
        output += f'g_vca = {status.g_vca}: '
        output += f'Current vacuum value on Channel A: {status.g_vca}\n'

        # g_vcb
        output += f'g_vcb = {status.g_vcb}: '
        output += f'Current vacuum value on Channel B: {status.g_vcb}\n'

        return output


def main(args=None):
    rclpy.init(args=args)
    node = OnRobotVGStatusListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

