#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "canopen_interfaces/srv/co_write.hpp"

void change_vel(const std::shared_ptr<rclcpp::Node>& node, int32_t vel_value) {
    // Client for /first_shaft_joint/sdo_write service
    auto sdo_write_client_1 = node->create_client<canopen_interfaces::srv::COWrite>("/first_shaft_joint/sdo_write");
    auto sdo_write_client_2 = node->create_client<canopen_interfaces::srv::COWrite>("/second_shaft_joint/sdo_write");
    auto sdo_write_client_3 = node->create_client<canopen_interfaces::srv::COWrite>("/third_shaft_joint/sdo_write");
    auto sdo_write_client_4 = node->create_client<canopen_interfaces::srv::COWrite>("/fourth_shaft_joint/sdo_write");

    // Prepare requests for sdo_write services
    auto sdo_write_request_1 = std::make_shared<canopen_interfaces::srv::COWrite::Request>();
    sdo_write_request_1->index = 0x6081;         // Velocity Object
    sdo_write_request_1->subindex = 0x00;        // Subindex 0
    sdo_write_request_1->data = vel_value;       // Use the parameter value

    // Call the /sdo_write services asynchronously
    auto sdo_write_result_1 = sdo_write_client_1->async_send_request(sdo_write_request_1);
    auto sdo_write_result_2 = sdo_write_client_2->async_send_request(sdo_write_request_1);
    auto sdo_write_result_3 = sdo_write_client_3->async_send_request(sdo_write_request_1);
    auto sdo_write_result_4 = sdo_write_client_4->async_send_request(sdo_write_request_1);

    RCLCPP_INFO(node->get_logger(), "Sent velocity value: %d", vel_value);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("changeVel");

    // Declare and retrieve the "VelValue" parameter
    node->declare_parameter<int32_t>("VelValue", 50); // Default value is 100
    int32_t vel_value;
    node->get_parameter("VelValue", vel_value);

    RCLCPP_INFO(node->get_logger(), "Velocity parameter set to: %d", vel_value);

    // Call the function with the retrieved parameter
    change_vel(node, vel_value);

    rclcpp::shutdown();
    return 0;
}
