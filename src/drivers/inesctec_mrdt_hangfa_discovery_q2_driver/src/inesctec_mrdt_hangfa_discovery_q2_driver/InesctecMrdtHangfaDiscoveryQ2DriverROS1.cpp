#include "inesctec_mrdt_hangfa_discovery_q2_driver/InesctecMrdtHangfaDiscoveryQ2DriverROS1.h"

namespace inesctec_mrdt_hangfa_discovery_q2_driver
{

InesctecMrdtHangfaDiscoveryQ2DriverROS1::
    InesctecMrdtHangfaDiscoveryQ2DriverROS1()
    : nh_priv_("~"), serial_comms_first_fault_(true)
{
  try
  {
    getParam();
  }
  catch (std::exception &e)
  {
    ROS_FATAL("Error reading the node parameters (%s)", e.what());

    ros::shutdown();

    return;
  }

  sample_time_ = ros::Time::now();

  pub_mot_enc_ =
      nh_.advertise<inesctec_mrdt_drivers_interfaces::MotEncArrayROS1>(
          "motors_enc", 10);

  sub_mot_ref_ =
      nh_.subscribe<inesctec_mrdt_drivers_interfaces::MotRefArrayROS1>(
          "motors_ref", 1, &InesctecMrdtHangfaDiscoveryQ2DriverROS1::subMotRef,
          this);

  rob_.setSerialPortName(serial_port_name_);
  rob_.openSerial();

  rob_.run = std::bind(&InesctecMrdtHangfaDiscoveryQ2DriverROS1::run, this);
  rob_.init();

  serial_port_timer_ = nh_.createTimer(
      ros::Duration(1),
      boost::bind(&InesctecMrdtHangfaDiscoveryQ2DriverROS1::checkSerialComms,
                  this));
}

void InesctecMrdtHangfaDiscoveryQ2DriverROS1::getParam()
{
  nh_priv_.param("encoder_res", encoder_res_, 48.0);
  nh_priv_.param("gear_reduction", gear_reduction_, 64.0);
  nh_priv_.param<std::string>("serial_port_name", serial_port_name_,
                              "/dev/ttyACM0");

  for (auto &m : rob_.mot)
  {
    m.encoder_res = encoder_res_;

    m.gear_reduction = gear_reduction_;
  }

  ROS_INFO("[%s] Encoder resolution: %lf (ticks/rev)",
           ros::this_node::getName().c_str(), rob_.mot[0].encoder_res);

  ROS_INFO("[%s] Gear reduction ratio: %lf (n:1)",
           ros::this_node::getName().c_str(), rob_.mot[0].gear_reduction);

  ROS_INFO("[%s] Serial port: %s", ros::this_node::getName().c_str(),
           serial_port_name_.c_str());
}

void InesctecMrdtHangfaDiscoveryQ2DriverROS1::checkSerialComms()
{
  if (!rob_.isSerialOpen())
  {
    if (serial_comms_first_fault_)
    {
      serial_comms_first_fault_ = false;

      ROS_INFO(
          "[%s] Couldn't open the serial port %s. Will retry every second.",
          ros::this_node::getName().c_str(), serial_port_name_.c_str());
    }

    rob_.mtx_.lock();
    rob_.stopMotors();
    rob_.mtx_.unlock();

    rob_.closeSerial();
    rob_.openSerial();

    if (rob_.isSerialOpen())
    {
      serial_comms_first_fault_ = true;

      ROS_INFO("[%s] Opened serial port %s.", ros::this_node::getName().c_str(),
               serial_port_name_.c_str());

      rob_.init();
    }
  }
}

void InesctecMrdtHangfaDiscoveryQ2DriverROS1::run()
{
  try
  {
    if (ros::Duration(ros::Time::now() - sample_time_) >
        ros::Duration(kWatchdogMotWRef))
    {
      rob_.mtx_.lock();
      rob_.stopMotors();
      rob_.mtx_.unlock();
    }
  }
  catch (std::exception &e)
  {
    ROS_WARN("[%s] Not possible to check the driver timeout condition (%s)",
             ros::this_node::getName().c_str(), e.what());

    sample_time_ = ros::Time::now();

    return;
  }

  pubMotEnc();
}

void InesctecMrdtHangfaDiscoveryQ2DriverROS1::pubMotEnc()
{
  inesctec_mrdt_drivers_interfaces::MotEncArrayROS1 msg;

  msg.stamp = ros::Time::now();
  msg.mot_enc.resize(4);

  rob_.mtx_.lock();

  for (int i = 0; i < 4; i++)
  {
    msg.mot_enc[i].enc_delta = rob_.mot[i].getEncTicksDeltaPub();

    msg.mot_enc[i].ticks_per_rev =
        rob_.mot[i].encoder_res * rob_.mot[i].gear_reduction;

    msg.mot_enc[i].ang_speed = rob_.mot[i].w;
  }

  rob_.mtx_.unlock();

  pub_mot_enc_.publish(msg);
}

void InesctecMrdtHangfaDiscoveryQ2DriverROS1::subMotRef(
    const inesctec_mrdt_drivers_interfaces::MotRefArrayROS1::ConstPtr &msg)
{
  if (msg->ang_speed_ref.size() >= 4)
  {
    rob_.mtx_.lock();

    for (int i = 0; i < 4; i++)
    {
      rob_.mot[i].w_r = msg->ang_speed_ref[i].ref;
    }

    rob_.mtx_.unlock();

    sample_time_ = msg->stamp;
  }
}

}  // namespace inesctec_mrdt_hangfa_discovery_q2_driver
