#!/usr/bin/env python3
"""Checks that OnRobotRGTcpNode survives a failed Modbus transaction.

This mirrors the VG test of the same shape. pymodbus returns a
ModbusIOException instead of raising it when a transaction fails, so a failed
read must not take the node down, must not be published as a made up status,
and must not drop a pending command.
"""

import importlib.util
import os
import sys
import types

import pytest

REPO = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def install_stubs():
    """Provides pymodbus and the generated messages when they are not
       installed, so the test also runs outside the gripper container.
    """
    try:
        from pymodbus.client import ModbusTcpClient  # noqa: F401
    except Exception:
        pymodbus = types.ModuleType('pymodbus')
        client = types.ModuleType('pymodbus.client')

        class ModbusTcpClient:
            def __init__(self, *args, **kwargs):
                pass

        client.ModbusTcpClient = ModbusTcpClient
        pymodbus.client = client
        sys.modules['pymodbus'] = pymodbus
        sys.modules['pymodbus.client'] = client

    try:
        from onrobot_rg_msgs.msg import OnRobotRGInput  # noqa: F401
    except Exception:
        msgs = types.ModuleType('onrobot_rg_msgs')
        msg = types.ModuleType('onrobot_rg_msgs.msg')

        class OnRobotRGInput:
            def __init__(self):
                self.g_fof = 0
                self.g_gwd = 0
                self.g_sta = 0
                self.g_wdf = 0

        class OnRobotRGOutput:
            def __init__(self):
                self.r_gfr = 0
                self.r_gwd = 0
                self.r_ctr = 0

        msg.OnRobotRGInput = OnRobotRGInput
        msg.OnRobotRGOutput = OnRobotRGOutput
        msgs.msg = msg
        sys.modules['onrobot_rg_msgs'] = msgs
        sys.modules['onrobot_rg_msgs.msg'] = msg


install_stubs()
sys.path.insert(0, os.path.join(REPO, 'onrobot_rg_control'))
sys.path.insert(0, os.path.join(REPO, 'onrobot_rg_modbus_tcp'))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
import onrobot_rg_modbus_tcp.comModbusTcp  # noqa: E402

spec = importlib.util.spec_from_file_location(
    'OnRobotRGTcpNode',
    os.path.join(REPO, 'onrobot_rg_control', 'nodes', 'OnRobotRGTcpNode.py'))
tcp_node = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tcp_node)

# Register 10 carries g_sta, whose lowest bit means "busy". It is left clear
# so that the node is willing to send commands.
GOOD_REGISTERS = [0] * 18
GOOD_REGISTERS[0] = 100    # g_fof
GOOD_REGISTERS[9] = 800    # g_gwd
GOOD_REGISTERS[10] = 0     # g_sta, not busy
GOOD_REGISTERS[17] = 1     # g_wdf

COMMAND = [400, 800, 1]    # r_gfr, r_gwd, r_ctr


class FakeIOException(Exception):
    """Stands in for pymodbus.exceptions.ModbusIOException, which the client
       returns (not raises) when a transaction times out or the socket breaks.
    """


class FakeModbusClient:
    """Modbus client whose transactions can be made to fail on demand."""

    def __init__(self):
        self.registers = list(GOOD_REGISTERS)
        self.fail = False
        self.fail_write = False
        self.closes = 0
        self.connects = 0
        self.write_attempts = 0
        self.writes = []

    def connect(self):
        self.connects += 1
        return True

    def close(self):
        self.closes += 1

    def read_holding_registers(self, address, count, slave):
        if self.fail:
            return FakeIOException('Modbus Error: [Input/Output] no response')
        return types.SimpleNamespace(
            registers=list(self.registers), isError=lambda: False)

    def write_registers(self, address, values, slave):
        self.write_attempts += 1
        if self.fail or self.fail_write:
            return FakeIOException('Modbus Error: [Input/Output] no response')
        self.writes.append(list(values))
        return types.SimpleNamespace(isError=lambda: False)


class RecordingPublisher:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


@pytest.fixture
def gripper_node():
    """Brings up the node with a fake client in place of the real gripper."""
    fake = FakeModbusClient()
    pub = RecordingPublisher()
    communication = onrobot_rg_modbus_tcp.comModbusTcp.communication

    def connectToDevice(self, ip, port, changer_addr=65):
        self.client = fake
        self.changer_addr = changer_addr
        return True

    original_connect = communication.connectToDevice
    original_publisher = Node.create_publisher
    original_subscription = Node.create_subscription
    communication.connectToDevice = connectToDevice
    Node.create_publisher = lambda self, *a, **k: pub
    Node.create_subscription = lambda self, *a, **k: None

    rclpy.init()
    try:
        node = tcp_node.OnRobotRGTcpNode()
        yield node, fake, pub
        node.destroy_node()
    finally:
        rclpy.shutdown()
        communication.connectToDevice = original_connect
        Node.create_publisher = original_publisher
        Node.create_subscription = original_subscription


def spin_ticks(node, ticks):
    """Runs the 10 Hz timer callback through the executor, which is where an
       escaping exception would take the node down.
    """
    for _ in range(ticks):
        rclpy.spin_once(node, timeout_sec=0.5)


def test_failed_read_does_not_publish_or_kill_the_node(gripper_node):
    node, fake, pub = gripper_node

    # A working link publishes the values read from the device.
    spin_ticks(node, 2)
    assert len(pub.published) >= 1
    assert (pub.published[-1].g_fof, pub.published[-1].g_gwd) == (100, 800)

    # The link breaks: no status may be published while the read fails.
    fake.fail = True
    published_before = len(pub.published)
    spin_ticks(node, 5)
    assert len(pub.published) == published_before

    # The node is still spinning and has tried to re-establish the link.
    assert rclpy.ok()
    assert not node.timer.is_canceled()
    assert fake.closes >= 1 and fake.connects >= 1

    # The link comes back with a different reading.
    fake.fail = False
    fake.registers[0] = 250
    spin_ticks(node, 3)
    assert len(pub.published) > published_before
    assert pub.published[-1].g_fof == 250

    # Recovering must never command the gripper by itself.
    assert fake.writes == []


def test_successful_command_is_written_once(gripper_node):
    node, fake, pub = gripper_node

    # A command that reaches the device must not be repeated afterwards.
    node.message = list(COMMAND)
    spin_ticks(node, 4)
    assert fake.writes == [COMMAND]


def test_failed_command_is_retried_until_it_is_written(gripper_node):
    node, fake, pub = gripper_node

    # A command whose write fails must keep being retried rather than being
    # marked as sent and dropped.
    fake.fail_write = True
    node.message = list(COMMAND)
    spin_ticks(node, 3)
    assert fake.writes == []
    assert fake.write_attempts >= 3
    assert rclpy.ok()
    assert not node.timer.is_canceled()

    # It is written exactly once when writing recovers, and then stops.
    fake.fail_write = False
    spin_ticks(node, 1)
    assert fake.writes == [COMMAND]
    attempts = fake.write_attempts
    spin_ticks(node, 3)
    assert fake.writes == [COMMAND]
    assert fake.write_attempts == attempts


def test_restart_power_service_reaches_the_client(gripper_node):
    from std_srvs.srv import Trigger

    node, fake, pub = gripper_node

    # The service used to call a non-existent self.gripper and never returned
    # a usable response.
    response = node.restartPowerCycle(Trigger.Request(), Trigger.Response())
    assert response.success is True
    assert fake.writes == [[2]]

    # A failed power cycle is reported rather than raised.
    fake.fail_write = True
    response = node.restartPowerCycle(Trigger.Request(), Trigger.Response())
    assert response.success is False
    assert rclpy.ok()
