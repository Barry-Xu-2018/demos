// Copyright 2023 Sony Group Corporation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include "rcl_interfaces/srv/get_logger_levels.hpp"
#include "rcl_interfaces/srv/set_logger_levels.hpp"

using namespace std::chrono_literals;

class LoggerServiceNode : public rclcpp::Node
{
public:
  LoggerServiceNode()
  : Node("LoggerServiceNode", "demos", rclcpp::NodeOptions().enable_logger_service(true))
  {
    auto callback = [this](std_msgs::msg::String::ConstSharedPtr msg)-> void {
      RCLCPP_DEBUG(this->get_logger(), "%s log with DEBUG logger level.", msg->data.c_str());
      RCLCPP_INFO(this->get_logger(), "%s with INFO logger level.", msg->data.c_str());
      RCLCPP_WARN(this->get_logger(), "%s with WARN logger level.", msg->data.c_str());
      RCLCPP_ERROR(this->get_logger(), "%s with ERROR logger level.", msg->data.c_str());
    };

    sub_ = this->create_subscription<std_msgs::msg::String>("output", 10, callback);
  }

  ~LoggerServiceNode() = default;

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

class RemoteNode : public rclcpp::Node
{
public:
  RemoteNode()
  : Node("RemoteNode", "demos")
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("output", 10);
    logger_set_client_ = this->create_client<rcl_interfaces::srv::SetLoggerLevels>(
      "/demos/LoggerServiceNode/set_logger_levels");
    logger_get_client_ = this->create_client<rcl_interfaces::srv::GetLoggerLevels>(
      "/demos/LoggerServiceNode/get_logger_levels");    
  }

  ~RemoteNode() = default;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr get_pub()
  {
    return pub_;
  }

  bool is_all_clients_ready()
  {
    return logger_set_client_->wait_for_service(1s)
      && logger_get_client_->wait_for_service(1s);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Client<rcl_interfaces::srv::SetLoggerLevels>::SharedPtr logger_set_client_;
  rclcpp::Client<rcl_interfaces::srv::GetLoggerLevels>::SharedPtr logger_get_client_;
};

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto logger_service_node = std::make_shared<LoggerServiceNode>();
  auto remote_node = std::make_shared<RemoteNode>();

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(logger_service_node);
  executor.add_node(remote_node);

  std::thread thread([& executor](){
    executor.spin();
  });

  // By default, only output while logger level > Debug
  {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Output 1";
    remote_node->get_pub()->publish(std::move(msg));
  }

  if (! remote_node->is_all_clients_ready()) {
    std::cout << "Client cannot connect to logger services !" << std::endl;
    goto _EXIT;
  }

  const std::string logger_name = "/demos/LoggerServiceNode"

  {
    auto request = std::make_shared<rcl_interfaces::srv::SetLoggerLevels::Request>();
    auto logger_level = rcl_interfaces::msg::LoggerLevel();
    logger_level. = logger_name
    logger_level.
  }
  
  std::this_thread::sleep_for(2s);

_EXIT:
  executor.cancel();
  if (thread.joinable()) {
    thread.join();
  }

  rclcpp::shutdown();
  return 0;
}