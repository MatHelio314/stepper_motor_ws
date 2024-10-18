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
    slave_eds_path = os.path.join(
        get_package_share_directory("nanotec_pd4_can"), "config", "double-pd4", "PD4-C6018L4204-E-08.eds"
    )

    master_bin_path = os.path.join(
        get_package_share_directory("nanotec_pd4_can"),
        "config",
        "double-pd4",
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
                get_package_share_directory("nanotec_pd4_can"),
                "config",
                "double-pd4",
                "master.dcf",
            ),
            "master_bin": master_bin_path,
            "bus_config": os.path.join(
                get_package_share_directory("nanotec_pd4_can"),
                "config",
                "double-pd4",
                "bus.yml",
            ),
            "can_interface_name": "can0",
        }.items(),
    )

    ld.add_action(device_container)


    return ld