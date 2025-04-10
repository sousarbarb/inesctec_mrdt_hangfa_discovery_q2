#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace inesctec_mrdt_omnijoy_driver
{

class OmniJoyDriverROS2 : public rclcpp::Node
{
 private:

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      on_set_param_callback_handle_;

  int linearx_;
  int lineary_;
  int angular_;
  int deadman_axis_;
  int turbo_axis_;
  int turbo_up_axis_;
  int turbo_down_axis_;

  double l_scale_;
  double a_scale_;
  double l_turbo_scale_;
  double l_turbo_maxscale_;
  double a_turbo_scale_;
  double a_turbo_maxscale_;

  geometry_msgs::msg::Twist last_published_;

  std::mutex publish_mutex_;

  bool deadman_pressed_;
  bool turbo_pressed_;
  bool turbo_up_pressed_;
  bool turbo_down_pressed_;
  bool zero_twist_published_;

  rclcpp::TimerBase::SharedPtr timer_;

 public:

  OmniJoyDriverROS2();

 private:

  void readParam();

  void joyCallback(sensor_msgs::msg::Joy::SharedPtr msg);
  void publish();

  rcl_interfaces::msg::SetParametersResult onSetParamCallback(
      const std::vector<rclcpp::Parameter>& parameters);
};

}  // namespace inesctec_mrdt_omnijoy_driver
