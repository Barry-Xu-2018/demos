// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>
#include <sstream>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_tutorials_cpp/visibility_control.h"

#include "action_msgs/srv/cancel_goal.hpp"

namespace action_tutorials_cpp
{
class FibonacciActionClient : public rclcpp::Node
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  ACTION_TUTORIALS_CPP_PUBLIC
  explicit FibonacciActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("fibonacci_action_client", node_options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "fibonacci");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FibonacciActionClient::send_goal, this));

    // Create a timer to check if the action server is available
    this->server_check_timer_ = this->create_wall_timer(
      std::chrono::seconds(13),
      [this]() {
        if (this->client_ptr_->wait_for_action_server(std::chrono::seconds(0))) {
          if (in_progress_) {
            RCLCPP_WARN(
              this->get_logger(),
              "Action server is available again, incomplete action will be cancelled.");
            in_progress_ = false;
            auto future_cancel = this->client_ptr_->async_cancel_all_goals();
            // Create a thread to wait for the cancel to complete
            std::thread(
              [this, future_cancel]() {
                try {
                  auto cancel_response = future_cancel.get();
                  RCLCPP_INFO(
                    this->get_logger(),
                    "Retrun code %d cancelled %zu goals.", cancel_response->return_code,
                    cancel_response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE ?
                    cancel_response->goals_canceling.size() : 0);
                } catch (const std::exception & e) {
                  RCLCPP_ERROR(this->get_logger(), "Failed to cancel goals: %s", e.what());
                }
                RCLCPP_WARN(this->get_logger(), "Send goal again");
                this->send_goal();
              }).detach();
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "Action server is unavailable");
          // Continuously check until the server is back
          this->server_available_timer_->reset();
          this->check_available_count_ = 0;
        }
      });
    this->server_check_timer_->cancel();  // Start with the timer canceled

    // Repeat checking until the action server is available
    this->server_available_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        if (this->client_ptr_->wait_for_action_server(std::chrono::seconds(0))) {
          this->server_available_timer_->cancel();
          ++this->check_available_count_;
          RCLCPP_INFO(this->get_logger(), "Action server is now available after %u s",
            this->check_available_count_.load());
          RCLCPP_WARN(this->get_logger(), "Send goal again");
          this->send_goal();
        } else {
          ++this->check_available_count_;
        }
      });
    this->server_available_timer_->cancel();  // Start with the timer canceled
  }

  ACTION_TUTORIALS_CPP_PUBLIC
  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&FibonacciActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr server_check_timer_;
  rclcpp::TimerBase::SharedPtr server_available_timer_;
  std::atomic_bool in_progress_{false};
  std::atomic_uint check_available_count_{0};

  ACTION_TUTORIALS_CPP_LOCAL
  void goal_response_callback(GoalHandleFibonacci::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      in_progress_ = true;
      server_check_timer_->reset();  // Reset the timer to start checking client is blocked since
                                     // action server is broken.
    }
  }

  ACTION_TUTORIALS_CPP_LOCAL
  void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  }

  ACTION_TUTORIALS_CPP_LOCAL
  void result_callback(const GoalHandleFibonacci::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        in_progress_ = false;
        send_goal();
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        in_progress_ = false;
        send_goal();
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        in_progress_ = false;
        send_goal();
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    in_progress_ = false;
    rclcpp::shutdown();
  }
};  // class FibonacciActionClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionClient)
