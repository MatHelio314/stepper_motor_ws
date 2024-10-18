cmake_minimum_required(VERSION 3.8)
project(nanotec_pd4_can)

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
#find_package(canopen_utils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)


#generate_dcf(double-pd4)
cogen_dcf(double-pd4)

add_executable(position_tick_client src/position_tick_motor.cpp)
ament_target_dependencies(position_tick_client
rclcpp std_srvs canopen_interfaces std_msgs)

add_executable(slider_button_node src/slider_button_node.cpp)
target_link_libraries(slider_button_node Qt5::Widgets)
ament_target_dependencies(slider_button_node rclcpp std_msgs)

install(TARGETS
position_tick_client
DESTINATION lib/${PROJECT_NAME})

install(TARGETS
slider_button_node
DESTINATION lib/${PROJECT_NAME})

# install launch file
install(DIRECTORY
launch/
DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
endif()

ament_package()