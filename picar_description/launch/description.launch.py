#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import launch_ros
import os

# TODO - This file is not useful atm, as I am publishing this in the viz package also..
# What to do here? Hmmm


def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="picar_description"
    ).find("picar_description")
    default_model_path = os.path.join(pkg_share, "urdf/steer_bot.urdf.xacro")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=default_model_path,
                description="The urdf model file name (xacro supported)",
            ),
            #
            # The robot state publisher publishes static TF2 transformations,
            # extracted from the URDF description of the robot
            #
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro ",
                                LaunchConfiguration(
                                    "model",
                                ),
                            ]
                        ),
                    }
                ],
            ),
        ]
    )
