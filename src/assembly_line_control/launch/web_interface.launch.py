#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for Assembly Line Control web interface using ROS Bridge.
    Starts rosbridge_server and the web interface.
    """
    
    # Launch arguments
    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='1112',
        description='Port for rosbridge WebSocket server'
    )
    
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='1111',
        description='Port for Flask web server'
    )
    
    # ROS Bridge Server Node
    # Note: rosbridge_server uses command-line arguments, not parameters
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        arguments=['--port', LaunchConfiguration('rosbridge_port')],
    )
    
    # Web Interface Node (minimal - just serves HTML)
    web_interface_node = Node(
        package='assembly_line_control',
        executable='web_interface',
        name='web_interface',
        output='screen',
        parameters=[{
            'rosbridge_port': LaunchConfiguration('rosbridge_port'),
        }],
    )
    
    return LaunchDescription([
        rosbridge_port_arg,
        web_port_arg,
        rosbridge_node,
        web_interface_node,
    ])

