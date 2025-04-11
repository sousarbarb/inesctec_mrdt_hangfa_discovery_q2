#pragma once

#include <inesctec_mrdt_drivers_interfaces/MotEncArrayROS1.h>
#include <inesctec_mrdt_drivers_interfaces/MotRefArrayROS1.h>
#include <ros/ros.h>

#include "inesctec_mrdt_hangfa_discovery_q2_driver/InesctecMrdtHangfaDiscoveryQ2.h"

namespace inesctec_mrdt_hangfa_discovery_q2_driver
{

const double kWatchdogMotWRef = 0.5;

class InesctecMrdtHangfaDiscoveryQ2DriverROS1
{
 private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  ros::Publisher pub_mot_enc_;
  ros::Subscriber sub_mot_ref_;

  ros::Timer serial_port_timer_;

  ros::Time sample_time_;

  InesctecMrdtHangfaDiscoveryQ2 rob_;

  double encoder_res_;
  double gear_reduction_;
  std::string serial_port_name_;

  bool serial_comms_first_fault_;

 public:

  InesctecMrdtHangfaDiscoveryQ2DriverROS1();

 private:

  void getParam();

  void checkSerialComms();

  void run();

  void pubMotEnc();
  void subMotRef(
      const inesctec_mrdt_drivers_interfaces::MotRefArrayROS1::ConstPtr &msg);
};

}  // namespace inesctec_mrdt_hangfa_discovery_q2_driver
