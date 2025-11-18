#!/usr/bin/env python3

"""
Launch file for Assembly Line OS

This starts up everything you need:
- ROS Bridge WebSocket server (lets the web interface talk to ROS)
- Web Interface (serves the HTML/JS interface)
- Motor Controller (tracks motor positions and sends commands)
- Relay Controller (tracks relay states and sends commands)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Start up all the components of Assembly Line OS
    """
    
    # Let users customize the ports if they want
    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9090',
        description='Port for rosbridge WebSocket server'
    )
    
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='1111',
        description='Port for Flask web server'
    )
    
    # Start ROS Bridge so the web interface can talk to ROS
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        arguments=['--port', LaunchConfiguration('rosbridge_port')],
    )
    
    # Start the web server that serves the HTML interface
    web_interface_node = Node(
        package='assembly_line_control',
        executable='web_interface',
        name='web_interface',
        output='screen',
        parameters=[],
    )
    
    # Start the motor controller that talks to the Arduino
    motor_controller_node = Node(
        package='assembly_line_control',
        executable='motor_controller',
        name='motor_controller',
        output='screen',
        parameters=[],
    )
    
    # Start the relay controller that talks to the Arduino
    relay_controller_node = Node(
        package='assembly_line_control',
        executable='relay_controller',
        name='relay_controller',
        output='screen',
        parameters=[],
    )
    
    # Start the sensor controller that monitors sensors
    sensor_controller_node = Node(
        package='assembly_line_control',
        executable='sensor_controller',
        name='sensor_controller',
        output='screen',
        parameters=[],
    )
    
    return LaunchDescription([
        rosbridge_port_arg,
        web_port_arg,
        rosbridge_node,
        web_interface_node,
        motor_controller_node,
        relay_controller_node,
        sensor_controller_node,
    ])

