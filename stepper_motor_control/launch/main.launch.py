import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

from ament_index_python.packages import get_package_share_directory


## ros2 launch stepper_motor_control main.launch.py sim:=<value>

def generate_launch_description():

    bus_manager = Node(
        package='stepper_motor_control',  
        executable='bus_manager',  
        output='screen'
    )
        
    ## Arguments
    sim_arg = DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Run motors with visualization (sim:=true) or use normal mode hardware launch (sim:=false)'
        )
    sim = LaunchConfiguration('sim')

    ## Launch visualization
    launch_visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('stepper_motor_control'), 'launch', 'visualization_motor_control.launch.py')]),
        condition=IfCondition(sim),
    )

    delayed_launch_visualization = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=bus_manager,
            on_exit=[launch_visualization]
        )
    )

    ## or launch actual hardware
    launch_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('stepper_motor_control'), 'launch', 'normal_motor_control.launch.py')]),
        condition=UnlessCondition(sim),
    ) 

    delayed_launch_hardware = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=bus_manager,
            on_exit=[launch_hardware]
        )
    )


 
    ## Launch description
    return LaunchDescription([

        # Nodes
        bus_manager,

        # Arguments
        sim_arg,

        # Launch
        delayed_launch_visualization,
        delayed_launch_hardware,
    ])
    
