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

#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#include "demo_nodes_cpp/visibility_control.h"

namespace demo_nodes_cpp
{
  // Create a Listener class that subclasses the generic rclcpp::Node base class.
  // The main function below will instantiate the class as a ROS node.
  class ListenerLaserScan : public rclcpp::Node
  {
  public:
    DEMO_NODES_CPP_PUBLIC
    explicit ListenerLaserScan(const rclcpp::NodeOptions &options)
        : Node("listener_laserscan", options)
    {

      const rosidl_message_type_support_t *ts = nullptr;

      if ((ts =
               get_message_typesupport_handle(
                   get_msg_type_support(), rosidl_typesupport_introspection_cpp::typesupport_identifier)) != nullptr)
      {
        members_ =
            static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(ts->data);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "get_message_typesupport_handle() return error !");
      }

      // Create a callback function for when messages are received.
      // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
      setvbuf(stdout, NULL, _IONBF, BUFSIZ);
      auto callback =
          [this](sensor_msgs::msg::LaserScan::SharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "range[0] is [%f]", msg->ranges[0]);

        static bool isShowed = false;
        if (isShowed)
        {
          return;
        }

        RCLCPP_INFO(this->get_logger(),"sec: %d, nanosec: %u, frame_id: %s, angle_min: %f, angle_max: %f, angle_increment: %f, time_increment: %f, scan_time: %f, range_min: %f, range_max: %f, ranges: %zu, intensities: %zu",
          msg->header.stamp.sec,
          msg->header.stamp.nanosec,
          msg->header.frame_id.c_str(),
          msg->angle_min,
          msg->angle_max,
          msg->angle_increment,
          msg->time_increment,
          msg->scan_time,
          msg->range_min,
          msg->range_max,
          msg->ranges.size(),
          msg->intensities.size());

        isShowed = true;
        scan_member(members_, static_cast<void *>(msg.get()));
      };
      // Create a subscription to the topic which can be matched with one or more compatible ROS
      // publishers.
      // Note that not all publishers on the same topic with the same type will be compatible:
      // they must have compatible Quality of Service policies.
      sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, callback);
    }

    const rosidl_message_type_support_t *
    get_msg_type_support()
    {
      return rosidl_typesupport_cpp::get_message_type_support_handle<
          sensor_msgs::msg::LaserScan>();
    }

    void scan_member(const rosidl_typesupport_introspection_cpp::MessageMembers *members, void *msg)
    {
      uint64_t addr = reinterpret_cast<uint64_t>(msg);
      for (uint32_t i = 0; i < members->member_count_; ++i)
      {
        const auto *member = members->members_ + i;

        switch (member->type_id_)
        {
        case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
          RCLCPP_INFO(this->get_logger(), "%s: Bool, %u", member->name_, member->offset_);
          break;
        case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
        case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
          RCLCPP_INFO(this->get_logger(), "%s: BYTE/UINT8, %u", member->name_, member->offset_);
          break;
        case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
        case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
          RCLCPP_INFO(this->get_logger(), "%s: CHAR/INT8, %u", member->name_, member->offset_);
          break;
        case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32:
          {
            if (member->is_array_) {
              auto float_array = reinterpret_cast<std_msgs::msg::Float32MultiArray::_data_type *>(addr + member->offset_);
              if (float_array->size()) {
                RCLCPP_INFO(this->get_logger(), "%s: FLOAT32 array, %zu, [0]=%f", member->name_, float_array->size(), (*float_array)[0]);
              } else {
                RCLCPP_INFO(this->get_logger(), "%s: FLOAT32 array, %zu", member->name_, float_array->size());
              }
            } else {
              RCLCPP_INFO(this->get_logger(), "%s: FLOAT32, %f", member->name_, *reinterpret_cast<std_msgs::msg::Float32::_data_type *>(addr + member->offset_));
            }
          }
          break;
        case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64:
          RCLCPP_INFO(this->get_logger(), "%s: FLOAT64, %u", member->name_, member->offset_);
          break;
        case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
          RCLCPP_INFO(this->get_logger(), "%s: INT16, %u", member->name_, member->offset_);
          break;
        case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
          RCLCPP_INFO(this->get_logger(), "%s: UINT16, %u", member->name_, member->offset_);
          break;
        case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
          {
            RCLCPP_INFO(this->get_logger(), "%s: INT32, %d", member->name_, *reinterpret_cast<std_msgs::msg::Int32::_data_type *>(addr + member->offset_));
          }
          break;
        case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
          {
            RCLCPP_INFO(this->get_logger(), "%s: UINT32, %u", member->name_, *reinterpret_cast<std_msgs::msg::UInt32::_data_type *>(addr + member->offset_));
          }
          break;
        case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
          RCLCPP_INFO(this->get_logger(), "%s: FLOAT64, %u", member->name_, member->offset_);
          break;
        case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
          RCLCPP_INFO(this->get_logger(), "%s: FLOAT64, %u", member->name_, member->offset_);
          break;
        case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
          {
            auto string = reinterpret_cast<std_msgs::msg::String::_data_type *>(addr + member->offset_);
            RCLCPP_INFO(this->get_logger(), "%s: STRING, %s (len:%zu)", member->name_, string->c_str(), sizeof(*string));
          }
          break;
        case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
          RCLCPP_INFO(this->get_logger(), "%s: WSTRING, %u", member->name_, member->offset_);
          break;
        case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
        {
          RCLCPP_INFO(this->get_logger(), "%s: ROS_TYPE_MESSAGE", member->name_);
          auto sub_members = (const rosidl_typesupport_introspection_cpp::MessageMembers *)member->members_->data;
          if (!member->is_array_)
          {
            scan_member(sub_members, (void *)(addr + member->offset_));
          }
          else
          {
            size_t array_size = 0;

            if (member->array_size_ && !member->is_upper_bound_)
            {
              array_size = member->array_size_;
            }
            else
            {
              array_size = member->size_function((void *)(addr + member->offset_));
            }

            if (array_size != 0 && !member->get_function)
            {
              RCLCPP_INFO(this->get_logger(), "unexpected error: get_function function is null");
              return;
            }
            for (size_t index = 0; index < array_size; ++index)
            {
              scan_member(sub_members, member->get_function((void *)(addr + member->offset_), index));
            }
          }
        }
          break;
        default:
          throw std::runtime_error("unknown type");
        }
      }
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    const rosidl_typesupport_introspection_cpp::MessageMembers *members_;
  };

} // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::ListenerLaserScan)
