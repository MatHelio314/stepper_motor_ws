#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "canopen_interfaces/srv/co_write.hpp"
#include "std_srvs/srv/trigger.hpp"

void call_sdo_write_and_init(const std::shared_ptr<rclcpp::Node>& node) {
    // Client for /first_shaft_joint/sdo_write service
    auto sdo_write_client_1 = node->create_client<canopen_interfaces::srv::COWrite>("/first_shaft_joint/sdo_write");
    // Client for /second_shaft_joint/sdo_write service
    auto sdo_write_client_2 = node->create_client<canopen_interfaces::srv::COWrite>("/second_shaft_joint/sdo_write");

    // Wait for both /sdo_write services to be available
    if (!sdo_write_client_1->wait_for_service(std::chrono::seconds(5)) || 
        !sdo_write_client_2->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "/first_shaft_joint/sdo_write or /second_shaft_joint/sdo_write service not available.");
        return;
    }

    // Prepare requests for sdo_write services
    auto sdo_write_request_1 = std::make_shared<canopen_interfaces::srv::COWrite::Request>();
    sdo_write_request_1->index = 0x2800;        // 10240 in hexadecimal
    sdo_write_request_1->subindex = 0x01;       // Subindex 1
    sdo_write_request_1->data = 1953460066;     // Data to send

    auto sdo_write_request_2 = std::make_shared<canopen_interfaces::srv::COWrite::Request>();
    sdo_write_request_2->index = 0x2800;        // 10240 in hexadecimal
    sdo_write_request_2->subindex = 0x01;       // Subindex 1
    sdo_write_request_2->data = 1953460066;     // Data to send

    // Call the /sdo_write services asynchronously
    auto sdo_write_result_1 = sdo_write_client_1->async_send_request(sdo_write_request_1);
    auto sdo_write_result_2 = sdo_write_client_2->async_send_request(sdo_write_request_2);

    // Wait for both responses
    if (rclcpp::spin_until_future_complete(node, sdo_write_result_1) != rclcpp::FutureReturnCode::SUCCESS || 
        rclcpp::spin_until_future_complete(node, sdo_write_result_2) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call /first_shaft_joint/sdo_write or /second_shaft_joint/sdo_write service.");
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Successfully called both /first_shaft_joint/sdo_write and /second_shaft_joint/sdo_write services.");

    // Wait for a few seconds
    rclcpp::sleep_for(std::chrono::seconds(10));

    // Client for /first_shaft_joint/sdo_write limit switch
    auto sdo_write_client_limit_1 = node->create_client<canopen_interfaces::srv::COWrite>("/first_shaft_joint/sdo_write");
    // Client for /second_shaft_joint/sdo_write limit switch
    auto sdo_write_client_limit_2 = node->create_client<canopen_interfaces::srv::COWrite>("/second_shaft_joint/sdo_write");

    // Wait for both limit switch services to be available
    if (!sdo_write_client_limit_1->wait_for_service(std::chrono::seconds(5)) || 
        !sdo_write_client_limit_2->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "/first_shaft_joint/sdo_write or /second_shaft_joint/sdo_write limit service not available.");
        return;
    }

    // Prepare requests for sdo_write services (limit switch)
    auto sdo_write_request_limit_1 = std::make_shared<canopen_interfaces::srv::COWrite::Request>();
    sdo_write_request_limit_1->index = 0x3701;        // Limit switch activation
    sdo_write_request_limit_1->subindex = 0x00;
    sdo_write_request_limit_1->data = 0xFFFF;

    auto sdo_write_request_limit_2 = std::make_shared<canopen_interfaces::srv::COWrite::Request>();
    sdo_write_request_limit_2->index = 0x3701;        // Limit switch activation
    sdo_write_request_limit_2->subindex = 0x00;
    sdo_write_request_limit_2->data = 0xFFFF;

    // Call the /sdo_write services asynchronously
    auto sdo_write_result_limit_1 = sdo_write_client_limit_1->async_send_request(sdo_write_request_limit_1);
    auto sdo_write_result_limit_2 = sdo_write_client_limit_2->async_send_request(sdo_write_request_limit_2);

    // Wait for both responses
    if (rclcpp::spin_until_future_complete(node, sdo_write_result_limit_1) != rclcpp::FutureReturnCode::SUCCESS || 
        rclcpp::spin_until_future_complete(node, sdo_write_result_limit_2) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call /first_shaft_joint/sdo_write or /second_shaft_joint/sdo_write limit switch service.");
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Successfully called both /first_shaft_joint/sdo_write and /second_shaft_joint/sdo_write limit switch services.");

    // Client for /first_shaft_joint/init service
    auto init_client_1 = node->create_client<std_srvs::srv::Trigger>("/first_shaft_joint/init");
    // Client for /second_shaft_joint/init service
    auto init_client_2 = node->create_client<std_srvs::srv::Trigger>("/second_shaft_joint/init");

    // Wait for both init services to be available
    if (!init_client_1->wait_for_service(std::chrono::seconds(5)) || 
        !init_client_2->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "/first_shaft_joint/init or /second_shaft_joint/init service not available.");
        return;
    }

    // Prepare requests for init services
    auto init_request_1 = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto init_request_2 = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Call the /init services asynchronously
    auto init_result_1 = init_client_1->async_send_request(init_request_1);
    auto init_result_2 = init_client_2->async_send_request(init_request_2);

    // Wait for both responses
    if (rclcpp::spin_until_future_complete(node, init_result_1) != rclcpp::FutureReturnCode::SUCCESS || 
        rclcpp::spin_until_future_complete(node, init_result_2) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call /first_shaft_joint/init or /second_shaft_joint/init service.");
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Successfully called both /first_shaft_joint/init and /second_shaft_joint/init services.");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("Reset_Home_X");
    call_sdo_write_and_init(node);
    rclcpp::shutdown();
    return 0;
}
