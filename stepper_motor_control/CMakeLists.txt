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


# cogen_dcf(robot_control)
generate_dcf(robot_control)


add_executable(position_tick_client src/position_tick_motor.cpp)
ament_target_dependencies(position_tick_client
rclcpp std_srvs canopen_interfaces)


install(TARGETS
position_tick_client
DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
launch urdf meshes src
DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
endif()

ament_package()