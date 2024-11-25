#include <chrono>
#include <cstdlib>
#include <memory>
#include "canopen_interfaces/srv/co_target_double.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/int32.hpp"  // To subscribe to slider topic
#include "sensor_msgs/msg/joint_state.hpp"  // To subscribe to joint_states
#include "canopen_interfaces/srv/co_write.hpp"

double target_position_ = 0;  // Store slider value
double current_position_ = 0; // Current positions of motor
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

// Callback function for joint state subscription of Motor
void joint_state_callback_1(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!msg->position.empty()) {
        current_position_ = msg->position[0];  // Store current position of Motor 1
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor 3 current position: %.2f", current_position_);
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("position_tick_motor_Y_node");

    // Initialize clients 
    auto init_client_ = node->create_client<std_srvs::srv::Trigger>("/third_shaft_joint/init");
    auto mode_client_ = node->create_client<std_srvs::srv::Trigger>("/third_shaft_joint/position_mode");
    auto halt_client_ = node->create_client<std_srvs::srv::Trigger>("/third_shaft_joint/halt");
    auto target_client_ = node->create_client<canopen_interfaces::srv::COTargetDouble>("/third_shaft_joint/target");


    // Subscribe to the slider value topic and joint states
    auto slider_sub = node->create_subscription<std_msgs::msg::Int32>("slider_value_y", 10, slider_callback);
    auto stop_sub = node->create_subscription<std_msgs::msg::Int32>("stop_motor_Y", 10, stop_callback);
    auto joint_state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
        "/third_shaft_joint/joint_states", 10, joint_state_callback_1);

    // Wait for both motor services to be available
    auto wait_for_services = [&](auto client) {
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node->get_logger(), "Service wait interrupted. Exiting.");
                return false;
            }
            RCLCPP_INFO(node->get_logger(), "Waiting for services to be available...");
        }
        return true;
    };

    if (!wait_for_services(init_client_) || !wait_for_services(mode_client_))
        return 0;



    // // Set behavior of limit switch activation to no reaction 
    // auto sdo_write_client = node->create_client<canopen_interfaces::srv::COWrite>("/third_shaft_joint/sdo_write");
    // // Wait for the /sdo_write service to be available
    // if (!sdo_write_client->wait_for_service(std::chrono::seconds(5))) {
    //     RCLCPP_ERROR(node->get_logger(), "/third_shaft_joint/sdo_write service not available.");
    //     return false;
    // }

    // // Prepare request for sdo_write service
    // auto sdo_write_request = std::make_shared<canopen_interfaces::srv::COWrite::Request>();
    // sdo_write_request->index = 0x3701;        // 10240 in hexadecimal
    // sdo_write_request->subindex = 0x00;       // Subindex 1
    // sdo_write_request->data = 0x0006;     // Data to send

    // // Call the /sdo_write service
    // auto sdo_write_result = sdo_write_client->async_send_request(sdo_write_request);
    // if (rclcpp::spin_until_future_complete(node, sdo_write_result) != rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call /third_shaft_joint/sdo_write service.");
    //     return false;
    // }
    // RCLCPP_INFO(node->get_logger(), "Successfully called /third_shaft_joint/sdo_write service.");






    // Call init and mode services for both motors
    auto trigger_req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto call_service = [&](auto client, const std::string &name) {
        auto result = client->async_send_request(trigger_req);
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "%s service called successfully", name.c_str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to call %s service", name.c_str());
        }
    };

    call_service(init_client_, "Motor Init");
    call_service(mode_client_, "Motor Config");

    // Prepare to send target positions to both motors
    auto target_req_ = std::make_shared<canopen_interfaces::srv::COTargetDouble::Request>();

    // Function to send target to both motors
    auto send_target = [&](double target) {
        target_req_->target = target;

        auto res = target_client_->async_send_request(target_req_);

        if (rclcpp::spin_until_future_complete(node, res) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Set target: %.2f to Y motor", target);
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to send target to motor");
        }
    };

    target_position_= current_position_;
    // Main loop: synchronize both motors' movements
    while (rclcpp::ok() && stop_bool == 0) {

        send_target(target_position_);  // Send the same target to both motors

        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    send_target(current_position_);
    call_service(halt_client_, "Motor Halt");
    rclcpp::shutdown();


    return 0;
}


