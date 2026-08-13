from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the launch arguments
    ip_arg = DeclareLaunchArgument(
        'ip',
        default_value='192.168.1.1',
        description='IP address for OnRobot'
    )
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='502',
        description='Port for OnRobot'
    )
    changer_addr_arg = DeclareLaunchArgument(
        'changer_addr',
        default_value='65',
        description='Changer address for OnRobot'
    )
    dummy_arg = DeclareLaunchArgument(
        'dummy',
        default_value='false',
        description='Dummy mode for OnRobot'
    )

    # Define the nodes
    onrobot_vg_status_listener_node = Node(
        package='onrobot_vg_control',
        executable='OnRobotVGStatusListener.py',
        name='OnRobotVGStatusListener',
        output='screen',
        parameters=[{
            'onrobot.ip': LaunchConfiguration('ip'),
            'onrobot.port': LaunchConfiguration('port'),
            'onrobot.changer_addr': LaunchConfiguration('changer_addr'),
            'onrobot.dummy': LaunchConfiguration('dummy')
        }]
    )

    onrobot_vg_tcp_node = Node(
        package='onrobot_vg_control',
        executable='OnRobotVGTcpNode.py',
        name='OnRobotVGTcpNode',
        output='screen',
        parameters=[{
            'onrobot.ip': LaunchConfiguration('ip'),
            'onrobot.port': LaunchConfiguration('port'),
            'onrobot.changer_addr': LaunchConfiguration('changer_addr'),
            'onrobot.dummy': LaunchConfiguration('dummy')
        }]
    )

    # Create and return the launch description
    return LaunchDescription([
        ip_arg,
        port_arg,
        changer_addr_arg,
        dummy_arg,
        onrobot_vg_status_listener_node,
        onrobot_vg_tcp_node
    ])
