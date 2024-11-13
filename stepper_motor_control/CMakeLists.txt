cmake_minimum_required(VERSION 3.8)
project(stepper_motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(canopen REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(canopen_interfaces REQUIRED)
find_package(canopen_402_driver REQUIRED)
find_package(canopen_ros2_controllers REQUIRED)
#find_package(canopen_utils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

# Enable automatic MOC processing
set(CMAKE_AUTOMOC ON)


# cogen_dcf(robot_control)
generate_dcf(robot_control)


add_executable(bus_manager src/bus_manager.cpp)
ament_target_dependencies(bus_manager
rclcpp std_msgs)

add_executable(Reset_Home_Y src/Reset_Home_Y.cpp)
ament_target_dependencies(Reset_Home_Y
rclcpp std_srvs canopen_interfaces)

add_executable(Reset_Home_X src/Reset_Home_X.cpp)
ament_target_dependencies(Reset_Home_X
rclcpp std_srvs canopen_interfaces)

add_executable(Reset_Home_Z src/Reset_Home_Z.cpp)
ament_target_dependencies(Reset_Home_Z
rclcpp std_srvs canopen_interfaces)

add_executable(position_tick_client_X src/position_tick_motor_X.cpp)
ament_target_dependencies(position_tick_client_X
rclcpp std_srvs canopen_interfaces std_msgs sensor_msgs canopen_interfaces)

add_executable(position_tick_client_Y src/position_tick_motor_Y.cpp)
ament_target_dependencies(position_tick_client_Y
rclcpp std_srvs canopen_interfaces std_msgs sensor_msgs canopen_interfaces)

add_executable(position_tick_client_Z src/position_tick_motor_Z.cpp)
ament_target_dependencies(position_tick_client_Z
rclcpp std_srvs canopen_interfaces std_msgs sensor_msgs canopen_interfaces)

add_executable(motor_joint_states_gui src/motor_joint_states_gui.cpp)
target_link_libraries(motor_joint_states_gui Qt5::Core Qt5::Widgets)
ament_target_dependencies(motor_joint_states_gui
rclcpp sensor_msgs )
target_include_directories(motor_joint_states_gui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(torque_tick_client_X src/torque_tick_motor_X.cpp)
ament_target_dependencies(torque_tick_client_X
rclcpp std_srvs canopen_interfaces std_msgs sensor_msgs canopen_interfaces)

add_executable(check_motor_connection src/check_motor_connection.cpp)
ament_target_dependencies(check_motor_connection
rclcpp std_msgs)

add_executable(slider_button_node src/slider_button_node.cpp)
target_link_libraries(slider_button_node Qt5::Widgets)
ament_target_dependencies(slider_button_node rclcpp std_msgs )


install(TARGETS
position_tick_client_X
position_tick_client_Y
position_tick_client_Z
torque_tick_client_X
check_motor_connection
bus_manager
slider_button_node
Reset_Home_Y
Reset_Home_X
Reset_Home_Z
motor_joint_states_gui
DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
launch urdf meshes src
DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
endif()

ament_package()