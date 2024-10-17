import os
import sys

import launch
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.event_handlers import OnProcessStart
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    ld = LaunchDescription()

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("stepper_motor_control"),
                    "urdf",
                    "robot_controller",
                    "robot_controller.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_control_config = PathJoinSubstitution(
        [FindPackageShare("stepper_motor_control"), "config/robot_control", "ros2_controllers.yaml"]
    )

# CONTROL NODE ############
 

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_control_config],
        output="screen",
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[control_node])

# JOINT STATE BROADCASTER ############

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_broadcaster_spawner]
        )
    )

# FORWARD POSITION CONTROLLER ############
#This controller only works with the Cia402System plugin

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )


    delayed_forward_position_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[forward_position_controller_spawner]
        )
    )

# DIFF CONT ############
#This controller only works with the Cia402System plugin

    diff_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_cont_spawner= RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[diff_cont_spawner]
        )
    )

# ROBOT CONTROLLER first ############
#This controller only works with the Cia402System plugin

    robot_controller_first_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_controller_first", "--controller-manager", "/controller_manager"],
    )

    delayed_robot_controller_first = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[robot_controller_first_spawner]
        )
    )

# ROBOT CONTROLLER second ############
#This controller only works with the Cia402System plugin

    robot_controller_second_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_controller_second", "--controller-manager", "/controller_manager"],
    )

    delayed_robot_controller_second = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[robot_controller_second_spawner]
        )
    )

# ROBOT CONTROLLER THIRD ############
#This controller only works with the Cia402System plugin

    robot_controller_third_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_controller_third", "--controller-manager", "/controller_manager"],
    )

    delayed_robot_controller_third = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[robot_controller_third_spawner]
        )
    )

# joint_trajectory_controller ############
#This controller only works with the Cia402System plugin

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    delayed_joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[joint_trajectory_controller_spawner]
        )
    )


# ROBOT STATE PUBLISHER ############

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )



# DEVICE CONTAINER ############
# Run this if you want to run the motors without the node controller 
    slave_eds_path = os.path.join(
        get_package_share_directory("stepper_motor_control"), "config", "robot_control", "PD4-C6018L4204-E-08.eds"
    )

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

    #     # Rviz
    # rviz_config = os.path.join(robot_description), 'launch', 'rviz', 'view_model.rviz')
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config]
    # )



    # ld.add_action(control_node)
    ld.add_action(delayed_controller_manager)
    ld.add_action(delayed_joint_state_broadcaster_spawner)
    # ld.add_action(delayed_robot_controller_first)
    # ld.add_action(delayed_robot_controller_second)
    ld.add_action(delayed_robot_controller_third)
    # ld.add_action(delayed_joint_trajectory_controller)
    # ld.add_action(delayed_forward_position_controller_spawner)
    ld.add_action(robot_state_publisher_node)
    # ld.add_action(rviz)
    # ld.add_action(delayed_diff_cont_spawner)
    # ld.add_action(device_container)

    return ld