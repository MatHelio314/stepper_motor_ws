#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
namespace fs = std::filesystem;



// The BusManager overwrites the bus.yml file with first the DefaultConfig and then listens to the check_motor_connection node for 
//       the connected motors and writes the corresponding config parameters only for the connected motors, then it builds and terminates itself

class BusManager : public rclcpp::Node {
public:
    BusManager() : Node("bus_manager") {
        // Initialize subscriber to listen to motor states
        motor_state_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "motor_states", 10, std::bind(&BusManager::motorStateCallback, this, std::placeholders::_1));

        // Write the default configuration at the beginning of bus.yml
        writeDefaultConfig(output_file_);

        // Launch check_motor_connection node
        const std::string command = "ros2 run stepper_motor_control check_motor_connection";
        // Execute the command
        int result = std::system(command.c_str());
        // Check if the command succeeded
        if (result == 0) {
           std::cout << "Launch successful." << std::endl;
        } else {
            std::cerr << "Launch failed with error code: " << result << std::endl;
        }

    }

private:
    
    rclcpp::TimerBase::SharedPtr timer_;

    std::string find_package_path(const std::string& src_dir, const std::string& package_name) {
    for (const auto& entry : fs::recursive_directory_iterator(src_dir)) {
        if (entry.is_directory() && entry.path().filename() == package_name) {
            return entry.path().string();
        }
    }
    throw std::runtime_error("Package " + package_name + " not found in " + src_dir);
    }


    std::string workspace_src = "src";
    std::string package_name = "stepper_motor_control";

    // Find the package dynamically
    std::string package_path = find_package_path(workspace_src, package_name);

    // Construct the paths to the template and output files
    std::string template_file_ = package_path + "/config/robot_control/bus_template.yml";
    std::string output_file_ = package_path + "/config/robot_control/bus.yml";


    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motor_state_subscriber_;

    // Writes the default header to bus.yml
    void writeDefaultConfig(const std::string &output_file) {
        std::ofstream outputStream(output_file, std::ios::trunc);
        if (!outputStream.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s", output_file.c_str());
            return;
        }
        outputStream << "options:\n"
                     << "  dcf_path: \"@BUS_CONFIG_PATH@\"\n\n"
                     << "master:\n"
                     << "  node_id: 10\n"
                     << "  driver: \"ros2_canopen::MasterDriver\"\n"
                     << "  package: \"canopen_master_driver\"\n"
                     << "  sync_period: 10000\n\n";
        outputStream.close();
    }




    // Retrieves the content of a specified node section from the template file
    std::vector<std::string> getNodeConfig(const std::string &target_node) {
        std::ifstream templateStream(template_file_);
        std::vector<std::string> nodeConfig;
        bool in_target_node = false;
        std::string line;

        if (!templateStream.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open template file: %s", template_file_.c_str());
            return nodeConfig;
        }

        while (std::getline(templateStream, line)) {
            if (line.find(target_node + ":") != std::string::npos) {
                in_target_node = true;
                nodeConfig.push_back(line);
            } else if (in_target_node && (!line.empty() && line[0] != ' ')) {
                break;
            } else if (in_target_node) {
                nodeConfig.push_back(line);
            }
        }
        templateStream.close();
        return nodeConfig;
    }

    // Writes node configuration to bus.yml
    void writeNodeConfig(const std::vector<std::string> &nodeConfig) {
        std::ofstream outputStream(output_file_, std::ios::app);
        if (!outputStream.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s", output_file_.c_str());
            return;
        }
        for (const auto &line : nodeConfig) {
            outputStream << line << std::endl;
        }
        outputStream.close();
    }

    

    // Callback function to handle messages from /motor_states
    void motorStateCallback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received motor state message: '%s'", msg->data.c_str());

        // Check if message format is correct
        if (msg->data.find("Connected motors: ") != 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid message format");
            return;
        }

        // Extract motor IDs from the message (e.g., "Connected motors: 1, 2, 3")
        std::string motor_list = msg->data.substr(17);  // Skip "Connected motors: "
        std::istringstream iss(motor_list);
        std::string motor_id_str;
        std::vector<std::string> connected_nodes;

        while (std::getline(iss, motor_id_str, ',')) {
            int motor_id = std::stoi(motor_id_str);
            std::string target_node = getTargetNode(motor_id);
            if (!target_node.empty()) {
                connected_nodes.push_back(target_node);
            }
        }

        // Append configuration for each connected motor node
        for (const auto& target_node : connected_nodes) {
            std::vector<std::string> nodeConfig = getNodeConfig(target_node);
            if (!nodeConfig.empty()) {
                writeNodeConfig(nodeConfig);
                RCLCPP_INFO(this->get_logger(), "Configuration for %s has been written to %s", target_node.c_str(), output_file_.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "No configuration found for %s", target_node.c_str());
            }
        }

        const std::string command = "colcon build --packages-select stepper_motor_control";
        // Execute the command
        int result = std::system(command.c_str());
        // Check if the command succeeded
        if (result == 0) {
           std::cout << "Build successful." << std::endl;
        } else {
            std::cerr << "Build failed with error code: " << result << std::endl;
        }


        rclcpp::shutdown();
    }

    // Maps motor ID to target node name
    std::string getTargetNode(int motor_id) {
        switch (motor_id) {
            case 1: return "first_shaft_joint";
            case 2: return "second_shaft_joint";
            case 3: return "third_shaft_joint";
            case 4: return "fourth_shaft_joint";
            default: return "";
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BusManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
