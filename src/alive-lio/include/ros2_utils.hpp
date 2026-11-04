#pragma once
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <cassert>
#include <cmath>
#include <stdexcept>

namespace alive {
inline builtin_interfaces::msg::Time stamp_from_seconds(double seconds) {
  return rclcpp::Time(static_cast<int64_t>(std::llround(seconds * 1e9)), RCL_ROS_TIME);
}
// ROS 2 represents floating-point parameters as doubles, including float destinations.
template<class T>
void parameter(const rclcpp::Node::SharedPtr& node, const std::string& name,
               T& destination, const T& default_value) {
  destination = node->declare_parameter<T>(name, default_value);
}
template<>
inline void parameter<float>(const rclcpp::Node::SharedPtr& node, const std::string& name,
                             float& destination, const float& default_value) {
  destination = static_cast<float>(node->declare_parameter<double>(name, default_value));
}
}  // namespace alive
