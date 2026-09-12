#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot_vg_msgs.msg import OnRobotVGInput


class OnRobotBaseVG(Node):
    """Base class (communication protocol agnostic) for sending commands
    and receiving the status of the OnRobot VG gripper.
    """

    def __init__(self):
        super().__init__("onrobot_base_vg")
        # Initiate output message as an empty list
        self.message = []

        # Note: after the instantiation,
        # a ".client" member must be added to the object

    def verify_command(self, command):
        """Verifies that the value of each variable satisfy its limits."""

        # Verify that each variable is in its correct range
        command.r_vca = max(0, command.r_vca)
        command.r_vca = min(255, command.r_vca)
        command.r_vcb = max(0, command.r_vcb)
        command.r_vcb = min(255, command.r_vcb)

        # Verify that the selected mode number is available
        if command.r_mca not in [0x0000, 0x0100, 0x0200]:
            self.get_logger().error(
                "Select the mode number for ch A from "
                "0x0000 (release), 0x0100 (grip), or 0x0200 (idle)."
            )
            rclpy.shutdown()
        if command.r_mcb not in [0x0000, 0x0100, 0x0200]:
            self.get_logger().error(
                "Select the mode number for ch B from "
                "0x0000 (release), 0x0100 (grip), or 0x0200 (idle)."
            )
            rclpy.shutdown()

        # Return the modified command
        return command

    def refresh_command(self, command):
        """Updates the command which will be sent
        during the next send_command() call.
        """

        # Limit the value of each variable
        command = self.verify_command(command)

        self.message = [command.r_mca, command.r_vca, command.r_mcb, command.r_vcb]

    def send_command(self):
        """Sends the command to the Gripper."""

        self.client.send_command(self.message)

    def get_status(self):
        """Requests the status from the gripper and
        return it in the OnRobotVGInput msg type.
        Returns None if the gripper status could not be read.
        """

        # Acquire status from the Gripper
        status = self.client.get_status()
        if status is None:
            return None

        # Message to output
        message = OnRobotVGInput()

        # Assign the values to their respective variables
        message.g_vca = status[0]
        message.g_vcb = status[1]

        return message


def main(args=None):
    rclpy.init(args=args)
    onrobot_base_vg = OnRobotBaseVG()

    try:
        rclpy.spin(onrobot_base_vg)
    except KeyboardInterrupt:
        pass
    finally:
        onrobot_base_vg.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
