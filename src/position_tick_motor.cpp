#include <chrono>
#include <cstdlib>
#include <memory>

#include "canopen_interfaces/srv/co_target_double.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp" 

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("position_tick_motor_node");

  RCLCPP_INFO(node->get_logger(), "Position Tick Motor Node Started");

  auto init_client = node->create_client<std_srvs::srv::Trigger>("/second_shaft_joint/init");
  auto mode_client = node->create_client<std_srvs::srv::Trigger>("/second_shaft_joint/velocity_mode");
  auto target_client = node->create_client<canopen_interfaces::srv::COTargetDouble>("/second_shaft_joint/target");

  // Service availability checks (split into individual while loops)
  while (!init_client->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the init service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Init service not available, waiting again...");
  }

  while (!mode_client->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the mode service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Mode service not available, waiting again...");
  }

  while (!target_client->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the target service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Target service not available, waiting again...");
  }

  // Init service call
  auto trigger_req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result = init_client->async_send_request(trigger_req);
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Init service called successfully");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to call init service");
  }

  // Mode service call
  result = mode_client->async_send_request(trigger_req);
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Config position mode service called successfully");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to call config service");
  }

  RCLCPP_INFO(node->get_logger(), "Starting to send target values");

  auto target_req = std::make_shared<canopen_interfaces::srv::COTargetDouble::Request>();
  double target = 0.0;
  while (rclcpp::ok())
  {
    target_req->target = target;
    auto res = target_client->async_send_request(target_req);
    if (rclcpp::spin_until_future_complete(node, res) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(node->get_logger(), "Set Target: %.2f", target);
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to call target service");
    }
    rclcpp::sleep_for(std::chrono::seconds(1));

    target += 5.0;
    if (target >= 105.0) target = 0;
  }

  return 0;
}
