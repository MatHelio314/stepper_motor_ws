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


# ROBOT STATE PUBLISHER ############

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )


# SLIDER NODE #######

    qt_slider_node = Node(
        package='stepper_motor_control',  
        executable='slider_button_node',  
        output='screen'
    )
    

# RVIZ2

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )


    ld.add_action(delayed_controller_manager)
    ld.add_action(qt_slider_node)
    ld.add_action(rviz2)
    ld.add_action(delayed_joint_state_broadcaster_spawner)
    ld.add_action(robot_state_publisher_node)
    return ld