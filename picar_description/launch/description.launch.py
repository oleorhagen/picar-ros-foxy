#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    model_name = LaunchConfiguration("model_name", default="steer_bot.urdf.xacro")

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_name',
            default_value="steer_bot.urdf.xacro",
            description="The urdf model file name (xacro supported)"),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(["xacro ", model_name]),
            }]),
    ])
