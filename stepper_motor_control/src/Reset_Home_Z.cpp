#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "canopen_interfaces/srv/co_write.hpp"
#include "std_srvs/srv/trigger.hpp"

void call_sdo_write_and_init(const std::shared_ptr<rclcpp::Node>& node) {
    // Client for /fourth_shaft_joint/sdo_write service
    auto sdo_write_client = node->create_client<canopen_interfaces::srv::COWrite>("/fourth_shaft_joint/sdo_write");

// Reset motor

    // Wait for the /sdo_write service to be available
    if (!sdo_write_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "/fourth_shaft_joint/sdo_write service not available.");
        return;
    }

    // Prepare request for sdo_write service
    auto sdo_write_request = std::make_shared<canopen_interfaces::srv::COWrite::Request>();
    sdo_write_request->index = 0x2800;        // 10240 in hexadecimal
    sdo_write_request->subindex = 0x01;       // Subindex 1
    sdo_write_request->data = 1953460066;     // Data to send

    // Call the /sdo_write service
    auto sdo_write_result = sdo_write_client->async_send_request(sdo_write_request);
    if (rclcpp::spin_until_future_complete(node, sdo_write_result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call /fourth_shaft_joint/sdo_write service.");
        return;
    }
    RCLCPP_INFO(node->get_logger(), "Successfully called /fourth_shaft_joint/sdo_write service.");


    // Wait for a few seconds
    rclcpp::sleep_for(std::chrono::seconds(10));

// Set behavior of limit switch activation to no reaction 

    auto sdo_write_client_limit = node->create_client<canopen_interfaces::srv::COWrite>("/fourth_shaft_joint/sdo_write");

    // Wait for the /sdo_write service to be available
    if (!sdo_write_client_limit->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "/fourth_shaft_joint/sdo_write service not available.");
        return;
    }

    // Prepare request for sdo_write service
    auto sdo_write_request_limit = std::make_shared<canopen_interfaces::srv::COWrite::Request>();
    sdo_write_request_limit->index = 0x3701;        // 10240 in hexadecimal
    sdo_write_request_limit->subindex = 0x00;       // Subindex 1
    sdo_write_request_limit->data = 0xFFFF;     // Data to send

    // Call the /sdo_write service
    auto sdo_write_result_limit = sdo_write_client_limit->async_send_request(sdo_write_request_limit);
    if (rclcpp::spin_until_future_complete(node, sdo_write_result_limit) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call /fourth_shaft_joint/sdo_write service.");
        return;
    }
    RCLCPP_INFO(node->get_logger(), "Successfully called /fourth_shaft_joint/sdo_write service.");

// Call init service

    // Client for /fourth_shaft_joint/init service
    auto init_client = node->create_client<std_srvs::srv::Trigger>("/fourth_shaft_joint/init");

    // Wait for the /init service to be available
    if (!init_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "/fourth_shaft_joint/init service not available.");
        return;
    }

    // Prepare request for init service
    auto init_request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Call the /init service
    auto init_result = init_client->async_send_request(init_request);
    if (rclcpp::spin_until_future_complete(node, init_result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call /fourth_shaft_joint/init service.");
        return;
    }
    RCLCPP_INFO(node->get_logger(), "Successfully called /fourth_shaft_joint/init service.");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("Reset_Home_Z");
    call_sdo_write_and_init(node);
    rclcpp::shutdown();
    return 0;
}
