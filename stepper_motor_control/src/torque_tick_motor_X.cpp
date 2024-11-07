#include <chrono>
#include <cstdlib>
#include <memory>
#include "canopen_interfaces/srv/co_target_double.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/int32.hpp"  // To subscribe to slider topic
#include "sensor_msgs/msg/joint_state.hpp"  // To subscribe to joint_states

double target_torque_ = 0;  // Store slider value for torque
double current_torque_1 = 0, current_torque_2 = 0;  // Current torques of motor 1 and motor 2

// Callback function for slider subscription
void slider_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    target_torque_ = static_cast<double>(msg->data);  // Update target torque based on slider value
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received torque slider value: %d", msg->data);
}

// Callback function for joint state subscription of Motor 1
void joint_state_callback_1(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!msg->effort.empty()) {
        current_torque_1 = msg->effort[0];  // Store current torque of Motor 1
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor 1 current torque: %.2f", current_torque_1);
    }
}

// Callback function for joint state subscription of Motor 2
void joint_state_callback_2(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!msg->effort.empty()) {
        current_torque_2 = msg->effort[0];  // Store current torque of Motor 2
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor 2 current torque: %.2f", current_torque_2);
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("torque_tick_node_X");

    // Initialize clients for Motor 1 and Motor 2 services
    auto init_client_1 = node->create_client<std_srvs::srv::Trigger>("/first_shaft_joint/init");
    auto mode_client_1 = node->create_client<std_srvs::srv::Trigger>("/first_shaft_joint/torque_mode");
    auto target_client_1 = node->create_client<canopen_interfaces::srv::COTargetDouble>("/first_shaft_joint/target");

    auto init_client_2 = node->create_client<std_srvs::srv::Trigger>("/third_shaft_joint/init");
    auto mode_client_2 = node->create_client<std_srvs::srv::Trigger>("/third_shaft_joint/torque_mode");
    auto target_client_2 = node->create_client<canopen_interfaces::srv::COTargetDouble>("/third_shaft_joint/target");

    // Subscribe to the slider value topic and joint states for both motors
    auto slider_sub = node->create_subscription<std_msgs::msg::Int32>("torque_value_x", 10, slider_callback);
    auto joint_state_sub_1 = node->create_subscription<sensor_msgs::msg::JointState>(
        "/first_shaft_joint/joint_states", 10, joint_state_callback_1);
    auto joint_state_sub_2 = node->create_subscription<sensor_msgs::msg::JointState>(
        "/third_shaft_joint/joint_states", 10, joint_state_callback_2);

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

    // Call init and torque mode services for both motors
    auto trigger_req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto call_service = [&](auto client, const std::string &name) {
        auto result = client->async_send_request(trigger_req);
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "%s service called successfully", name.c_str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to call %s service", name.c_str());
        }
    };

    call_service(init_client_1, "Motor 1 Init");
    call_service(init_client_2, "Motor 2 Init");
    call_service(mode_client_1, "Motor 1 Torque Mode");
    call_service(mode_client_2, "Motor 2 Torque Mode");

    // Prepare to send target torques to both motors
    auto target_req_1 = std::make_shared<canopen_interfaces::srv::COTargetDouble::Request>();
    auto target_req_2 = std::make_shared<canopen_interfaces::srv::COTargetDouble::Request>();
    double target_torque_1 = current_torque_1, target_torque_2 = current_torque_2;

    // Function to send target torque to both motors
    auto send_targets = [&](double target_torque) {
        target_req_1->target = target_torque;
        target_req_2->target = target_torque;

        auto res1 = target_client_1->async_send_request(target_req_1);
        auto res2 = target_client_2->async_send_request(target_req_2);

        if (rclcpp::spin_until_future_complete(node, res1) == rclcpp::FutureReturnCode::SUCCESS &&
            rclcpp::spin_until_future_complete(node, res2) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Set target torque: %.2f to both motors", target_torque);
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to send target torques to motors");
        }
    };

    // Main loop: synchronize both motors' torque
    while (rclcpp::ok()) {
        send_targets(target_torque_1);  // Send the same target torque to both motors

        rclcpp::sleep_for(std::chrono::milliseconds(50));

        // Adjust the target torque towards the slider value if different
        if (target_torque_1 != target_torque_) {
            target_torque_1 += (target_torque_1 < target_torque_) ? 10.0 : -10.0;
            target_torque_2 = target_torque_1;  // Synchronize motor 2 with motor 1
            if (std::abs(target_torque_1 - target_torque_) < 10.0)
                target_torque_1 = target_torque_;
        }
    }

    return 0;
}
