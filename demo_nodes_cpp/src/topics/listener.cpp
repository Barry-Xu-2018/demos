// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "my_example_msgs/msg/test_data1_m.hpp"

#include "demo_nodes_cpp/visibility_control.h"

namespace demo_nodes_cpp
{
// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Listener : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit Listener(const rclcpp::NodeOptions & options)
  : Node("listener", options)
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto callback =
      //[this](const std_msgs::msg::Float64::SharedPtr msg) -> void
      //[this](const std_msgs::msg::Float64::UniquePtr msg) -> void
      [this](const my_example_msgs::msg::TestData1M::SharedPtr msg) -> void
      {
        //RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
        //RCLCPP_INFO(this->get_logger(), "I heard: [%f] %p", msg->data, static_cast<void *>(&(msg->data)));
        RCLCPP_INFO(
          this->get_logger(),
          "I heard: [%d] %p",
          static_cast<int>(msg->test_data[0]),
          static_cast<void *>(&(msg->test_data)));
      };
    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.
    //sub_ = create_subscription<std_msgs::msg::String>("chatter_pod", 10, callback);
    //sub_ = create_subscription<std_msgs::msg::Float64>("chatter_pod", 10, callback);
    sub_ = create_subscription<my_example_msgs::msg::TestData1M>("chatter_pod", 10, callback);

     RCLCPP_INFO(this->get_logger(), "Can loan %d", sub_->can_loan_messages()? 1: 0);
  }

private:
  //rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  rclcpp::Subscription<my_example_msgs::msg::TestData1M>::SharedPtr sub_;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::Listener)
