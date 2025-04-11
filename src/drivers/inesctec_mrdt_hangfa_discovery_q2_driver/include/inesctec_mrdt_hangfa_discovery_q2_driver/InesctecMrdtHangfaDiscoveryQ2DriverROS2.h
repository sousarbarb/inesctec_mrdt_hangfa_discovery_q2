#pragma once

#include <inesctec_mrdt_drivers_interfaces/msg/mot_enc_array.hpp>
#include <inesctec_mrdt_drivers_interfaces/msg/mot_ref_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "inesctec_mrdt_hangfa_discovery_q2_driver/InesctecMrdtHangfaDiscoveryQ2.h"

namespace inesctec_mrdt_hangfa_discovery_q2_driver
{

const double kWatchdogMotWRef = 0.5;

class InesctecMrdtHangfaDiscoveryQ2DriverROS2 : public rclcpp::Node
{
 private:

  rclcpp::Publisher<inesctec_mrdt_drivers_interfaces::msg::MotEncArray>::
      SharedPtr pub_mot_enc_;

  rclcpp::Subscription<inesctec_mrdt_drivers_interfaces::msg::MotRefArray>::
      SharedPtr sub_mot_ref_;

  rclcpp::TimerBase::SharedPtr serial_port_timer_;

  rclcpp::Time sample_time_;

  InesctecMrdtHangfaDiscoveryQ2 rob_;

  double encoder_res_;
  double gear_reduction_;
  std::string serial_port_name_;

  bool serial_comms_first_fault_;

 public:

  InesctecMrdtHangfaDiscoveryQ2DriverROS2();

 private:

  void getParam();

  void checkSerialComms();

  void run();

  void pubMotEnc();
  void subMotRef(
      const inesctec_mrdt_drivers_interfaces::msg::MotRefArray::SharedPtr msg);
};

}  // namespace inesctec_mrdt_hangfa_discovery_q2_driver
