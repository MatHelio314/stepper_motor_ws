#include <chrono>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <cstdlib>
#include <regex>
#include <thread>
#include <future>
#include <fstream> 
#include <array>
#include <filesystem>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


// The MotorConnectionChecker node creates a .txt file that stores the candump of the can0 interface, then sends commands to each motors. 
// It then closes the candump command and reads the file looking for the answers, if it finds an expected answer in the file it means that 
// the corresponding motor is connected, it then publishes "Connected motors : 1, 2...." depending on the connected motors and then terminates itself 

namespace fs = std::filesystem;
using namespace std::chrono_literals;

class MotorConnectionChecker : public rclcpp::Node {
public:
    MotorConnectionChecker() : Node("motor_connection_checker") {
        // Motor node IDs to check
        node_ids_ = {1, 2, 3, 4};
        can_interface_ = "can0";  // Use the CAN interface

        // Publisher for motor states
        motor_state_publisher_ = this->create_publisher<std_msgs::msg::String>("motor_states", 10);

        // Periodically check connections every 1 second
        // timer_ = this->create_wall_timer(1000ms, std::bind(&MotorConnectionChecker::checkConnections, this));

        // Just once
        checkConnections();

        // The node gracefully terminates itself
        rclcpp::shutdown();
    }

private:
    std::vector<int> node_ids_;
    std::string can_interface_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_state_publisher_;

    void checkConnections() {
        // Start candump in a separate thread
        std::string log_filename = generateFilename();

        // Start the candump process and capture its PID
        pid_t candump_pid = startCandump(log_filename);

        // Send commands to all motors
        for (const auto& node_id : node_ids_) {
            sendCanCommand(node_id);
        }

        // Wait for a moment to allow responses to be logged
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Adjust timing if necessary

        // Ends the candump command (to empty the buffer into the file)
        killCandump(candump_pid);

        // Check the log file for responses
        std::vector<int> connected_motors;
        for (const auto& node_id : node_ids_) {
            bool is_connected = checkMotorResponse(log_filename, node_id);
            if (is_connected) {
                connected_motors.push_back(node_id);
            }
        }

        // Publish the overall motor connection status
        publishMotorStatus(connected_motors);

        // Cleanup the log file after checking
        fs::remove(log_filename);
    }

    pid_t startCandump(const std::string& log_filename) {
        // Start the candump process
        std::string command = "candump -l -L " + can_interface_ + " > " + log_filename + " & echo $!";
        FILE* pipe = popen(command.c_str(), "r");
        if (!pipe) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start candump.");
            return -1;
        }

        // Read the PID of the candump process
        char buffer[128];
        fgets(buffer, sizeof(buffer), pipe);
        pclose(pipe);
        return static_cast<pid_t>(std::stoi(buffer));
    }

    void killCandump(pid_t pid) {
        if (pid > 0) {
            // Kill the candump process
            std::string kill_command = "kill " + std::to_string(pid);
            system(kill_command.c_str());
            RCLCPP_INFO(this->get_logger(), "Stopped candump process with PID %d.", pid);
        }
    }

    // the candump -l command stores it's data in a specific file which has the following name : 
    std::string generateFilename() {
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm* now_tm = std::localtime(&now_c);
        char filename[100];
        std::strftime(filename, sizeof(filename), "candump-%Y-%m-%d_%H%M%S.log", now_tm);

        return filename;
    }

    void sendCanCommand(int node_id) {
        std::string command = "cansend " + can_interface_ + " " + getCanCommand(node_id);
        int send_result = system(command.c_str());

        if (send_result != 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to send CAN message for Motor %d.", node_id);
        }
    }

    std::string getCanCommand(int node_id) {
        // Command to request motor's status
        std::stringstream command;
        command << std::hex << (0x600 + node_id) << "#40.41.60.00.00.00.00.00";  // Adjust based on specific motor SDO read command
        return command.str();
    }

    bool checkMotorResponse(const std::string& log_filename, int node_id) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Increase delay if needed

        // Check if the file exists and has content
        if (!std::filesystem::exists(log_filename)) {
            RCLCPP_ERROR(this->get_logger(), "Log file does not exist for Motor %d.", node_id);
            return false;
        }

        auto file_size = std::filesystem::file_size(log_filename);
        if (file_size == 0) {
            RCLCPP_WARN(this->get_logger(), "Log file is empty for Motor %d.", node_id);
            return false;
        }

        std::ifstream log_file(log_filename);
        if (!log_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open log file for Motor %d.", node_id);
            return false;
        }

        std::string line;
        std::stringstream expected_response;
        expected_response << std::hex << (0x580 + node_id);  // Expected response ID for the node

        bool found_response = false;

        while (std::getline(log_file, line)) {
            if (line.find(expected_response.str() + "#4B") != std::string::npos) {
                found_response = true;
                break;
            }
        }

        log_file.close();

        if (found_response) {
            RCLCPP_INFO(this->get_logger(), "Motor %d is connected.", node_id);
        } else {
            RCLCPP_WARN(this->get_logger(), "Motor %d NOT connected.", node_id);
        }

        return found_response;
    }

    void publishMotorStatus(const std::vector<int>& connected_motors) {
        std_msgs::msg::String msg;

        // Create CSV format message
        std::string message = "Connected motors: ";
        for (const auto& motor : connected_motors) {
            message += std::to_string(motor) + ", ";
        }
        if (!connected_motors.empty()) {
            message = message.substr(0, message.size() - 2); // Remove trailing comma
        } else {
            message += "None";
        }
        
        msg.data = message;

        motor_state_publisher_->publish(msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorConnectionChecker>());
    rclcpp::shutdown();
    return 0;
}
