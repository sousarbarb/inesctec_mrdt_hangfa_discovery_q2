#include "inesctec_mrdt_omnijoy_driver/OmniJoyDriverROS2.h"

namespace inesctec_mrdt_omnijoy_driver
{

using namespace std::chrono_literals;

OmniJoyDriverROS2::OmniJoyDriverROS2() : Node("inesctec_mrdt_omnijoy_driver")
{
  try
  {
    readParam();
  }
  catch (std::exception& e)
  {
    RCLCPP_FATAL(this->get_logger(), "Error reading the node parameters (%s)",
                 e.what());

    rclcpp::shutdown();

    return;
  }

  deadman_pressed_ = false;
  turbo_pressed_ = false;
  turbo_up_pressed_ = false;
  turbo_down_pressed_ = false;
  zero_twist_published_ = false;

  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&OmniJoyDriverROS2::joyCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
      100ms, std::bind(&OmniJoyDriverROS2::publish, this));

  on_set_param_callback_handle_ =
      this->add_on_set_parameters_callback(std::bind(
          &OmniJoyDriverROS2::onSetParamCallback, this, std::placeholders::_1));
}

void OmniJoyDriverROS2::readParam()
{
  linearx_ = this->declare_parameter<int>("axis_linear_x", 1);
  lineary_ = this->declare_parameter<int>("axis_linear_y", 2);
  angular_ = this->declare_parameter<int>("axis_angular", 0);
  deadman_axis_ = this->declare_parameter<int>("axis_deadman", 4);
  turbo_axis_ = this->declare_parameter<int>("axis_turbo", 5);
  turbo_up_axis_ = this->declare_parameter<int>("axis_turbo_up", 6);
  turbo_down_axis_ = this->declare_parameter<int>("axis_turbo_down", 7);

  l_scale_ = this->declare_parameter<double>("scale_linear", 0.1);
  a_scale_ = this->declare_parameter<double>("scale_angular", 0.2);
  l_turbo_scale_ = this->declare_parameter<double>("turbo_scale_linear", 0.2);
  l_turbo_maxscale_ =
      this->declare_parameter<double>("turbo_max_scale_linear", 0.4);
  a_turbo_scale_ = this->declare_parameter<double>("turbo_scale_angular", 0.4);
  a_turbo_maxscale_ =
      this->declare_parameter<double>("turbo_max_scale_angular", 0.8);

  RCLCPP_INFO(this->get_logger(), "axis_linear_x: %d", linearx_);
  RCLCPP_INFO(this->get_logger(), "axis_linear_y: %d", lineary_);
  RCLCPP_INFO(this->get_logger(), "axis_angular: %d", angular_);
  RCLCPP_INFO(this->get_logger(), "axis_deadman: %d", deadman_axis_);
  RCLCPP_INFO(this->get_logger(), "axis_turbo: %d", turbo_axis_);
  RCLCPP_INFO(this->get_logger(), "axis_turbo_up: %d", turbo_up_axis_);
  RCLCPP_INFO(this->get_logger(), "axis_turbo_down: %d", turbo_down_axis_);

  RCLCPP_INFO(this->get_logger(), "scale_linear: %lf (m/s)", l_scale_);
  RCLCPP_INFO(this->get_logger(), "scale_angular: %lf (rad/s)", a_scale_);
  RCLCPP_INFO(this->get_logger(), "turbo_scale_linear: %lf (m/s)",
              l_turbo_maxscale_);
  RCLCPP_INFO(this->get_logger(), "turbo_max_scale_linear: %lf (m/s)",
              l_turbo_scale_);
  RCLCPP_INFO(this->get_logger(), "turbo_scale_angular: %lf (rad/s)",
              a_turbo_scale_);
  RCLCPP_INFO(this->get_logger(), "turbo_max_scale_angular: %lf (rad/s)",
              a_turbo_maxscale_);
}

void OmniJoyDriverROS2::joyCallback(sensor_msgs::msg::Joy::SharedPtr msg)
{
  geometry_msgs::msg::Twist vel;

  vel.linear.x = msg->axes[linearx_];
  vel.linear.y = msg->axes[lineary_];
  vel.linear.z = 0.0;

  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = msg->axes[angular_];

  last_published_ = vel;

  deadman_pressed_ = msg->buttons[deadman_axis_];
  turbo_pressed_ = msg->buttons[turbo_axis_];
  turbo_up_pressed_ = msg->buttons[turbo_up_axis_];
  turbo_down_pressed_ = msg->buttons[turbo_down_axis_];
}

void OmniJoyDriverROS2::publish()
{
  std::lock_guard<std::mutex> lock(publish_mutex_);

  if (turbo_up_pressed_ && !turbo_down_pressed_)
  {
    l_turbo_scale_ = l_turbo_scale_ + 0.05;
    a_turbo_scale_ = a_turbo_scale_ + 0.10;

    if (l_turbo_scale_ > l_turbo_maxscale_)
    {
      l_turbo_scale_ = l_turbo_maxscale_;
    }

    if (a_turbo_scale_ > a_turbo_maxscale_)
    {
      a_turbo_scale_ = a_turbo_maxscale_;
    }
  }
  if (!turbo_up_pressed_ && turbo_down_pressed_)
  {
    l_turbo_scale_ = l_turbo_scale_ - 0.05;
    a_turbo_scale_ = a_turbo_scale_ - 0.10;

    if (l_turbo_scale_ < l_scale_)
    {
      l_turbo_scale_ = l_scale_;
    }

    if (a_turbo_scale_ < a_scale_)
    {
      a_turbo_scale_ = a_scale_;
    }
  }

  if (deadman_pressed_ && !turbo_pressed_)
  {
    last_published_.linear.x = last_published_.linear.x * l_scale_;
    last_published_.linear.y = last_published_.linear.y * l_scale_;
    last_published_.linear.z = 0.0;

    last_published_.angular.x = 0.0;
    last_published_.angular.y = 0.0;
    last_published_.angular.z = last_published_.angular.z * a_scale_;

    vel_pub_->publish(last_published_);

    zero_twist_published_ = false;
  }
  else if (deadman_pressed_ && turbo_pressed_)
  {
    last_published_.linear.x = last_published_.linear.x * l_turbo_scale_;
    last_published_.linear.y = last_published_.linear.y * l_turbo_scale_;
    last_published_.linear.z = 0.0;

    last_published_.angular.x = 0.0;
    last_published_.angular.y = 0.0;
    last_published_.angular.z = last_published_.angular.z * a_turbo_scale_;

    vel_pub_->publish(last_published_);

    zero_twist_published_ = false;
  }
  else if (!deadman_pressed_ && !zero_twist_published_)
  {
    geometry_msgs::msg::Twist zero_msg;

    zero_msg.linear.x = 0.0;
    zero_msg.linear.y = 0.0;
    zero_msg.linear.z = 0.0;

    zero_msg.angular.x = 0.0;
    zero_msg.angular.y = 0.0;
    zero_msg.angular.z = 0.0;

    vel_pub_->publish(zero_msg);

    zero_twist_published_ = true;
  }
}

rcl_interfaces::msg::SetParametersResult OmniJoyDriverROS2::onSetParamCallback(
    const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (auto& param : parameters)
  {
    if ((param.get_name() == "axis_linear_x") ||
        (param.get_name() == "axis_linear_y") ||
        (param.get_name() == "axis_angular") ||
        (param.get_name() == "axis_deadman") ||
        (param.get_name() == "axis_turbo") ||
        (param.get_name() == "axis_turbo_up") ||
        (param.get_name() == "axis_turbo_down"))
    {
      try
      {
        if (param.get_value<int>() < 0)
        {
          result.successful = false;
          result.reason = "joy.axes ID must be greater or equal to 0";

          RCLCPP_ERROR(this->get_logger(),
                       "Error when validating the axis ID parameter %s "
                       "(error: %s)",
                       param.get_name().c_str(), result.reason.c_str());

          return result;
        }
      }
      catch (std::exception& e)
      {
        result.successful = false;
        result.reason = e.what();

        RCLCPP_ERROR(this->get_logger(),
                     "Error when validating the axis ID parameter %s "
                     "(error: %s)",
                     param.get_name().c_str(), e.what());

        return result;
      }
    }

    if ((param.get_name() == "scale_linear") ||
        (param.get_name() == "scale_angular") ||
        (param.get_name() == "turbo_scale_linear") ||
        (param.get_name() == "turbo_max_scale_linear") ||
        (param.get_name() == "turbo_scale_angular") ||
        (param.get_name() == "turbo_max_scale_angular"))
    {
      try
      {
        if (param.get_value<double>() < 0)
        {
          result.successful = false;
          result.reason = "scale parameters must be greater or equal to 0";

          RCLCPP_ERROR(this->get_logger(),
                       "Error when validating the scale parameter %s "
                       "(error: %s)",
                       param.get_name().c_str(), result.reason.c_str());

          return result;
        }
      }
      catch (std::exception& e)
      {
        result.successful = false;
        result.reason = e.what();

        RCLCPP_ERROR(this->get_logger(),
                     "Error when validating the scale parameter %s "
                     "(error: %s)",
                     param.get_name().c_str(), e.what());

        return result;
      }
    }

    if (param.get_name() == "axis_linear_x")
    {
      linearx_ = param.get_value<int>();
    }
    else if (param.get_name() == "axis_linear_y")
    {
      lineary_ = param.get_value<int>();
    }
    else if (param.get_name() == "axis_angular")
    {
      angular_ = param.get_value<int>();
    }
    else if (param.get_name() == "axis_deadman")
    {
      deadman_axis_ = param.get_value<int>();
    }
    else if (param.get_name() == "axis_turbo")
    {
      turbo_axis_ = param.get_value<int>();
    }
    else if (param.get_name() == "axis_turbo_up")
    {
      turbo_up_axis_ = param.get_value<int>();
    }
    else if (param.get_name() == "axis_turbo_down")
    {
      turbo_down_axis_ = param.get_value<int>();
    }
    else if (param.get_name() == "scale_linear")
    {
      l_scale_ = param.get_value<double>();
    }
    else if (param.get_name() == "scale_angular")
    {
      a_scale_ = param.get_value<double>();
    }
    else if (param.get_name() == "turbo_scale_linear")
    {
      l_turbo_scale_ = param.get_value<double>();
    }
    else if (param.get_name() == "turbo_max_scale_linear")
    {
      l_turbo_maxscale_ = param.get_value<double>();
    }
    else if (param.get_name() == "turbo_scale_angular")
    {
      a_turbo_scale_ = param.get_value<double>();
    }
    else if (param.get_name() == "turbo_max_scale_angular")
    {
      a_turbo_maxscale_ = param.get_value<double>();
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Ignoring unkown parameter '%s'",
                  param.get_name().c_str());
    }
  }

  RCLCPP_INFO(this->get_logger(), "axis_linear_x: %d", linearx_);
  RCLCPP_INFO(this->get_logger(), "axis_linear_y: %d", lineary_);
  RCLCPP_INFO(this->get_logger(), "axis_angular: %d", angular_);
  RCLCPP_INFO(this->get_logger(), "axis_deadman: %d", deadman_axis_);
  RCLCPP_INFO(this->get_logger(), "axis_turbo: %d", turbo_axis_);
  RCLCPP_INFO(this->get_logger(), "axis_turbo_up: %d", turbo_up_axis_);
  RCLCPP_INFO(this->get_logger(), "axis_turbo_down: %d", turbo_down_axis_);

  RCLCPP_INFO(this->get_logger(), "scale_linear: %lf (m/s)", l_scale_);
  RCLCPP_INFO(this->get_logger(), "scale_angular: %lf (rad/s)", a_scale_);
  RCLCPP_INFO(this->get_logger(), "turbo_scale_linear: %lf (m/s)",
              l_turbo_maxscale_);
  RCLCPP_INFO(this->get_logger(), "turbo_max_scale_linear: %lf (m/s)",
              l_turbo_scale_);
  RCLCPP_INFO(this->get_logger(), "turbo_scale_angular: %lf (rad/s)",
              a_turbo_scale_);
  RCLCPP_INFO(this->get_logger(), "turbo_max_scale_angular: %lf (rad/s)",
              a_turbo_maxscale_);

  return result;

  /* Source:
   * https://roboticsbackend.com/ros2-rclcpp-parameter-callback/
   * https://github.com/ros2/demos/blob/rolling/demo_nodes_cpp/src/parameters/set_parameters_callback.cpp
   *    (the last one has pre, on and post set callback handlers)
   */
}

}  // namespace inesctec_mrdt_omnijoy_driver
