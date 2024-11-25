#include <chrono>
#include <cstdlib>
#include <memory>
#include "canopen_interfaces/srv/co_target_double.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/int32.hpp"  // To subscribe to slider topic
#include "sensor_msgs/msg/joint_state.hpp"  // To subscribe to joint_states

double target_position_ = 0;  // Store slider value, applied to both motors
double current_position_1 = 0, current_position_2 = 0;  // Current positions of motor 1 and motor 2
double stop_bool = 0;

// Callback function for slider subscription
void slider_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    target_position_ = static_cast<double>(msg->data);  // Update target based on slider value
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received slider value: %d", msg->data);
}

void stop_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    stop_bool = static_cast<double>(msg->data);  // Update target based on slider value
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received stop value: %d", msg->data);
}

// Callback function for joint state subscription of Motor 1
void joint_state_callback_1(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!msg->position.empty()) {
        current_position_1 = msg->position[0];  // Store current position of Motor 1
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor 1 current position: %.2f", current_position_1);
    }
}

// Callback function for joint state subscription of Motor 2
void joint_state_callback_2(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!msg->position.empty()) {
        current_position_2 = msg->position[0];  // Store current position of Motor 2
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor 2 current position: %.2f", current_position_2);
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("dual_motor_synchronized_node");

    // Initialize clients for Motor 1 and Motor 2 services
    auto init_client_1 = node->create_client<std_srvs::srv::Trigger>("/first_shaft_joint/init");
    auto mode_client_1 = node->create_client<std_srvs::srv::Trigger>("/first_shaft_joint/position_mode");
    auto halt_client_1 = node->create_client<std_srvs::srv::Trigger>("/first_shaft_joint/halt");
    auto target_client_1 = node->create_client<canopen_interfaces::srv::COTargetDouble>("/first_shaft_joint/target");

    auto init_client_2 = node->create_client<std_srvs::srv::Trigger>("/second_shaft_joint/init");
    auto mode_client_2 = node->create_client<std_srvs::srv::Trigger>("/second_shaft_joint/position_mode");
    auto halt_client_2 = node->create_client<std_srvs::srv::Trigger>("/second_shaft_joint/halt");
    auto target_client_2 = node->create_client<canopen_interfaces::srv::COTargetDouble>("/second_shaft_joint/target");

    // Subscribe to the slider value topic and joint states for both motors
    auto slider_sub = node->create_subscription<std_msgs::msg::Int32>("slider_value_x", 10, slider_callback);
    auto stop_sub = node->create_subscription<std_msgs::msg::Int32>("stop_motor_X", 10, stop_callback);
    auto joint_state_sub_1 = node->create_subscription<sensor_msgs::msg::JointState>(
        "/first_shaft_joint/joint_states", 10, joint_state_callback_1);
    auto joint_state_sub_2 = node->create_subscription<sensor_msgs::msg::JointState>(
        "/second_shaft_joint/joint_states", 10, joint_state_callback_2);

    // Wait for both motor services to be available
    auto wait_for_services = [&](auto client1, auto client2) {
        while (!client1->wait_for_service(std::chrono::seconds(1)) ||
               !client2->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node->get_logger(), "Service wait interrupted. Exiting.");
                return false;
            }
            RCLCPP_INFO(node->get_logger(), "Waiting for services to be available...");
        }
        return true;
    };

    if (!wait_for_services(init_client_1, init_client_2) || !wait_for_services(mode_client_1, mode_client_2))
        return 0;

    // Function to call a service synchronously for both motors
    auto call_service_sync = [&](auto client1, auto client2, const std::string &service_name) {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result1 = client1->async_send_request(request);
        auto result2 = client2->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node, result1) == rclcpp::FutureReturnCode::SUCCESS &&
            rclcpp::spin_until_future_complete(node, result2) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "%s service called successfully on both motors", service_name.c_str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to call %s service on both motors", service_name.c_str());
            rclcpp::shutdown();
        }
    };

    // Call init and mode services for both motors
    // call_service_sync(init_client_1, init_client_2, "Init");
    call_service_sync(mode_client_1, mode_client_2, "Position Mode");

    // Prepare to send target positions to both motors
    auto target_req_1 = std::make_shared<canopen_interfaces::srv::COTargetDouble::Request>();
    auto target_req_2 = std::make_shared<canopen_interfaces::srv::COTargetDouble::Request>();

    // Function to send target positions to both motors
    auto send_targets = [&](double target, double current_position_1, double current_position_2) {
        if (std::abs(current_position_1 - current_position_2) > 200) {
            target_req_1->target = current_position_1;
            target_req_2->target = current_position_1;
        } else {
            target_req_1->target = target;
            target_req_2->target = target;      
        }

        auto res1 = target_client_1->async_send_request(target_req_1);
        auto res2 = target_client_2->async_send_request(target_req_2);

        if (std::abs(current_position_1 - current_position_2) > 200) {
            rclcpp::sleep_for(std::chrono::milliseconds(1000));
        }

        if (rclcpp::spin_until_future_complete(node, res1) == rclcpp::FutureReturnCode::SUCCESS &&
            rclcpp::spin_until_future_complete(node, res2) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Set target: %.2f to both motors", target);
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to send targets to motors");
        }
    };

    target_position_ = current_position_1;

    // Main loop: synchronize both motors' movements
    while (rclcpp::ok() && stop_bool == 0) {
        send_targets(target_position_, current_position_1, current_position_2);
    }

    // Halt motors and shut down
    target_req_1->target = current_position_1;
    target_req_2->target = current_position_1;
    auto res1 = target_client_1->async_send_request(target_req_1);
    auto res2 = target_client_2->async_send_request(target_req_2);
    call_service_sync(halt_client_1, halt_client_2, "Halt");
    rclcpp::shutdown();

    return 0;
}
