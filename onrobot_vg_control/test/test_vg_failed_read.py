#!/usr/bin/env python3
"""Checks that OnRobotVGTcpNode survives a failed Modbus read.

pymodbus returns a ModbusIOException instead of raising it when a transaction
fails, so a failed read used to take the node down with an AttributeError on
'.registers'. The node must instead stay alive, publish nothing at all while
the read fails (a fabricated status would read as "no vacuum" downstream),
leave the gripper uncommanded, and publish again once the link comes back.
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
        from onrobot_vg_msgs.msg import OnRobotVGInput  # noqa: F401
    except Exception:
        msgs = types.ModuleType('onrobot_vg_msgs')
        msg = types.ModuleType('onrobot_vg_msgs.msg')

        class OnRobotVGInput:
            def __init__(self):
                self.g_vca = 0
                self.g_vcb = 0

        class OnRobotVGOutput:
            def __init__(self):
                self.r_mca = 0
                self.r_vca = 0
                self.r_mcb = 0
                self.r_vcb = 0

        msg.OnRobotVGInput = OnRobotVGInput
        msg.OnRobotVGOutput = OnRobotVGOutput
        msgs.msg = msg
        sys.modules['onrobot_vg_msgs'] = msgs
        sys.modules['onrobot_vg_msgs.msg'] = msg


install_stubs()
sys.path.insert(0, os.path.join(REPO, 'onrobot_vg_control'))
sys.path.insert(0, os.path.join(REPO, 'onrobot_vg_modbus_tcp'))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402

spec = importlib.util.spec_from_file_location(
    'OnRobotVGTcpNode',
    os.path.join(REPO, 'onrobot_vg_control', 'nodes', 'OnRobotVGTcpNode.py'))
tcp_node = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tcp_node)


class FakeIOException(Exception):
    """Stands in for pymodbus.exceptions.ModbusIOException, which the client
       returns (not raises) when a transaction times out or the socket breaks.
    """


class FakeModbusClient:
    """Modbus client whose transactions can be made to fail on demand."""

    def __init__(self):
        self.registers = [12, 34]
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

    def connect_to_device(self, ip, port, changer_addr=65):
        self.client = fake
        self.changer_addr = changer_addr

    original_connect = tcp_node.Communication.connect_to_device
    original_publisher = Node.create_publisher
    original_subscription = Node.create_subscription
    tcp_node.Communication.connect_to_device = connect_to_device
    Node.create_publisher = lambda self, *a, **k: pub
    Node.create_subscription = lambda self, *a, **k: None

    rclpy.init()
    try:
        node = tcp_node.OnRobotVGTcp()
        yield node, fake, pub
        node.destroy_node()
    finally:
        rclpy.shutdown()
        tcp_node.Communication.connect_to_device = original_connect
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
    assert (pub.published[-1].g_vca, pub.published[-1].g_vcb) == (12, 34)

    # The link breaks: no status may be published while the read fails.
    fake.fail = True
    published_before = len(pub.published)
    spin_ticks(node, 5)
    assert len(pub.published) == published_before

    # The node is still spinning and has tried to re-establish the link.
    assert rclpy.ok()
    assert not node.timer.is_canceled()
    assert fake.closes >= 1 and fake.connects >= 1

    # The link comes back with a different vacuum reading.
    fake.fail = False
    fake.registers = [56, 78]
    spin_ticks(node, 3)
    assert len(pub.published) > published_before
    assert (pub.published[-1].g_vca, pub.published[-1].g_vcb) == (56, 78)

    # Recovering must never command the gripper by itself.
    assert fake.writes == []


def test_failed_write_does_not_kill_the_node(gripper_node):
    node, fake, pub = gripper_node

    # A command is pending and the write to the device fails.
    fake.fail_write = True
    node.gripper.message = [0x0100, 255, 0x0100, 255]
    spin_ticks(node, 3)
    assert fake.writes == []

    # Reading still works, so the status keeps being published, the node is
    # still spinning, and the link has been re-established.
    assert len(pub.published) >= 1
    assert rclpy.ok()
    assert not node.timer.is_canceled()
    assert fake.closes >= 1 and fake.connects >= 1

    # Once writing works again the pending command reaches the device.
    fake.fail_write = False
    spin_ticks(node, 2)
    assert fake.writes == [[0x0100 + 255, 0x0100 + 255]]


def test_successful_command_is_written_once(gripper_node):
    node, fake, pub = gripper_node

    # A command that reaches the device must not be repeated afterwards.
    node.gripper.message = [0x0100, 255, 0x0100, 255]
    spin_ticks(node, 4)
    assert fake.writes == [[0x0100 + 255, 0x0100 + 255]]


def test_failed_command_is_retried_until_it_is_written(gripper_node):
    node, fake, pub = gripper_node

    # A command whose write fails must keep being retried: the repetition is
    # the only retry the write path has.
    fake.fail_write = True
    node.gripper.message = [0x0100, 255, 0x0100, 255]
    spin_ticks(node, 3)
    assert fake.writes == []
    assert fake.write_attempts >= 3

    # It is written exactly once when writing recovers, and then stops.
    fake.fail_write = False
    spin_ticks(node, 1)
    assert fake.writes == [[0x0100 + 255, 0x0100 + 255]]
    attempts = fake.write_attempts
    spin_ticks(node, 3)
    assert fake.writes == [[0x0100 + 255, 0x0100 + 255]]
    assert fake.write_attempts == attempts
