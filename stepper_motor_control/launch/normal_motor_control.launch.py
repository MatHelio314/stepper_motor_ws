import os
import sys

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    master_bin_path = os.path.join(
        get_package_share_directory("stepper_motor_control"),
        "config",
        "robot_control",
        "master.bin",
    )
    if not os.path.exists(master_bin_path):
        master_bin_path = ""


    device_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_core"), "launch"),
                "/canopen.launch.py",
            ]
        ),
        launch_arguments={
            "master_config": os.path.join(
                get_package_share_directory("stepper_motor_control"),
                "config",
                "robot_control",
                "master.dcf",
            ),
            "master_bin": master_bin_path,
            "bus_config": os.path.join(
                get_package_share_directory("stepper_motor_control"),
                "config",
                "robot_control",
                "bus.yml",
            ),
            "can_interface_name": "can0",
        }.items(),
    )

    # SLIDER NODE #######

    qt_slider_node = Node(
        package='stepper_motor_control',  
        executable='slider_button_node',  
        output='screen'
    )

    ld.add_action(device_container)
    ld.add_action(qt_slider_node)

    return ld