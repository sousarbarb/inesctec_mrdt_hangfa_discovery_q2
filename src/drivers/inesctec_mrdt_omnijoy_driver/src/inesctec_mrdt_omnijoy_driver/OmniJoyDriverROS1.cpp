#include "inesctec_mrdt_omnijoy_driver/OmniJoyDriverROS1.h"

namespace inesctec_mrdt_omnijoy_driver
{

OmniJoyDriverROS1::OmniJoyDriverROS1() : nh_priv_("~")
{
  try
  {
    readParam();
  }
  catch (std::exception& e)
  {
    ROS_FATAL("Error reading the node parameters (%s)", e.what());

    ros::shutdown();

    return;
  }

  deadman_pressed_ = false;
  turbo_pressed_ = false;
  turbo_up_pressed_ = false;
  turbo_down_pressed_ = false;
  zero_twist_published_ = false;

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(
      "joy", 10, &OmniJoyDriverROS1::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.100),
                           boost::bind(&OmniJoyDriverROS1::publish, this));

  dynamic_reconfigure::Server<
      inesctec_mrdt_omnijoy_driver::OmniJoyParamConfig>::CallbackType callback;

  callback = boost::bind(&OmniJoyDriverROS1::cfgServerCallback, this, _1, _2);

  cfg_server_.setCallback(callback);
}

void OmniJoyDriverROS1::readParam()
{
  nh_priv_.param("axis_linear_x", linearx_, 1);
  nh_priv_.param("axis_linear_y", lineary_, 2);
  nh_priv_.param("axis_angular", angular_, 0);
  nh_priv_.param("axis_deadman", deadman_axis_, 4);
  nh_priv_.param("axis_turbo", turbo_axis_, 5);
  nh_priv_.param("axis_turbo_up", turbo_up_axis_, 6);
  nh_priv_.param("axis_turbo_down", turbo_down_axis_, 7);

  nh_priv_.param("scale_linear", l_scale_, 0.1);
  nh_priv_.param("scale_angular", a_scale_, 0.2);
  nh_priv_.param("turbo_scale_linear", l_turbo_scale_, 0.2);
  nh_priv_.param("turbo_max_scale_linear", l_turbo_maxscale_, 0.4);
  nh_priv_.param("turbo_scale_angular", a_turbo_scale_, 0.4);
  nh_priv_.param("turbo_max_scale_angular", a_turbo_maxscale_, 0.8);

  /*   ROS_INFO("[%s] axis_linear_x: %d",
        ros::this_node::getName().c_str(), linearx_);
    ROS_INFO("[%s] axis_linear_y: %d",
        ros::this_node::getName().c_str(), lineary_);
    ROS_INFO("[%s] axis_angular: %d",
        ros::this_node::getName().c_str(), angular_);
    ROS_INFO("[%s] axis_deadman: %d",
        ros::this_node::getName().c_str(), deadman_axis_);
    ROS_INFO("[%s] axis_turbo: %d",
        ros::this_node::getName().c_str(), turbo_axis_);
    ROS_INFO("[%s] axis_turbo_up: %d",
        ros::this_node::getName().c_str(), turbo_up_axis_);
    ROS_INFO("[%s] axis_turbo_down: %d",
        ros::this_node::getName().c_str(), turbo_down_axis_);
    ROS_INFO("[%s] scale_linear: %lf (m/s)",
        ros::this_node::getName().c_str(), l_scale_);
    ROS_INFO("[%s] scale_angular: %lf (rad/s)",
        ros::this_node::getName().c_str(), a_scale_);
    ROS_INFO("[%s] turbo_scale_linear: %lf (m/s)",
        ros::this_node::getName().c_str(), l_turbo_maxscale_);
    ROS_INFO("[%s] turbo_max_scale_linear: %lf (m/s)",
        ros::this_node::getName().c_str(), l_turbo_scale_);
    ROS_INFO("[%s] turbo_scale_angular: %lf (rad/s)",
        ros::this_node::getName().c_str(), a_turbo_scale_);
    ROS_INFO("[%s] turbo_max_scale_angular: %lf (rad/s)",
        ros::this_node::getName().c_str(), a_turbo_maxscale_); */
}

void OmniJoyDriverROS1::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  geometry_msgs::Twist vel;

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

void OmniJoyDriverROS1::publish()
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

    vel_pub_.publish(last_published_);

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

    vel_pub_.publish(last_published_);

    zero_twist_published_ = false;
  }
  else if (!deadman_pressed_ && !zero_twist_published_)
  {
    geometry_msgs::Twist zero_msg;

    zero_msg.linear.x = 0.0;
    zero_msg.linear.y = 0.0;
    zero_msg.linear.z = 0.0;

    zero_msg.angular.x = 0.0;
    zero_msg.angular.y = 0.0;
    zero_msg.angular.z = 0.0;

    vel_pub_.publish(zero_msg);

    zero_twist_published_ = true;
  }
}

void OmniJoyDriverROS1::cfgServerCallback(
    inesctec_mrdt_omnijoy_driver::OmniJoyParamConfig& config, uint32_t level)
{
  linearx_ = config.axis_linear_x;
  lineary_ = config.axis_linear_y;
  angular_ = config.axis_angular;
  deadman_axis_ = config.axis_deadman;
  turbo_axis_ = config.axis_turbo;
  turbo_up_axis_ = config.axis_turbo_up;
  turbo_down_axis_ = config.axis_turbo_down;

  l_scale_ = config.scale_linear;
  a_scale_ = config.scale_angular;
  l_turbo_scale_ = config.turbo_scale_linear;
  l_turbo_maxscale_ = config.turbo_max_scale_linear;
  a_turbo_scale_ = config.turbo_scale_angular;
  a_turbo_maxscale_ = config.turbo_max_scale_angular;

  ROS_INFO("[%s] axis_linear_x: %d", ros::this_node::getName().c_str(),
           linearx_);
  ROS_INFO("[%s] axis_linear_y: %d", ros::this_node::getName().c_str(),
           lineary_);
  ROS_INFO("[%s] axis_angular: %d", ros::this_node::getName().c_str(),
           angular_);
  ROS_INFO("[%s] axis_deadman: %d", ros::this_node::getName().c_str(),
           deadman_axis_);
  ROS_INFO("[%s] axis_turbo: %d", ros::this_node::getName().c_str(),
           turbo_axis_);
  ROS_INFO("[%s] axis_turbo_up: %d", ros::this_node::getName().c_str(),
           turbo_up_axis_);
  ROS_INFO("[%s] axis_turbo_down: %d", ros::this_node::getName().c_str(),
           turbo_down_axis_);
  ROS_INFO("[%s] scale_linear: %lf (m/s)", ros::this_node::getName().c_str(),
           l_scale_);
  ROS_INFO("[%s] scale_angular: %lf (rad/s)", ros::this_node::getName().c_str(),
           a_scale_);
  ROS_INFO("[%s] turbo_scale_linear: %lf (m/s)",
           ros::this_node::getName().c_str(), l_turbo_maxscale_);
  ROS_INFO("[%s] turbo_max_scale_linear: %lf (m/s)",
           ros::this_node::getName().c_str(), l_turbo_scale_);
  ROS_INFO("[%s] turbo_scale_angular: %lf (rad/s)",
           ros::this_node::getName().c_str(), a_turbo_scale_);
  ROS_INFO("[%s] turbo_max_scale_angular: %lf (rad/s)",
           ros::this_node::getName().c_str(), a_turbo_maxscale_);
}

}  // namespace inesctec_mrdt_omnijoy_driver
