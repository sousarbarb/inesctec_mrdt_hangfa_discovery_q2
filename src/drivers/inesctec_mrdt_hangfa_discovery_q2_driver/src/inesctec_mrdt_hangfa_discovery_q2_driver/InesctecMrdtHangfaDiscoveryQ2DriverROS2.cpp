#include "inesctec_mrdt_hangfa_discovery_q2_driver/InesctecMrdtHangfaDiscoveryQ2DriverROS2.h"

namespace inesctec_mrdt_hangfa_discovery_q2_driver
{

  using namespace std::chrono_literals;

  InesctecMrdtHangfaDiscoveryQ2DriverROS2::InesctecMrdtHangfaDiscoveryQ2DriverROS2()
      : Node("inesctec_mrdt_hangfa_discovery_q2_driver"), serial_comms_first_fault_(true)
  {

    try
    {
      getParam();
    }
    catch (std::exception &e)
    {
      RCLCPP_FATAL(this->get_logger(),
                   "Error reading the node parameters (%s)", e.what());

      rclcpp::shutdown();

      return;
    }

    sample_time_ = this->now();

    pub_mot_enc_ = this->create_publisher<inesctec_mrdt_drivers_interfaces::msg::MotEncArray>("motors_enc", 10);

    sub_mot_ref_ = this->create_subscription<inesctec_mrdt_drivers_interfaces::msg::MotRefArray>(
        "motors_ref", 1,
        std::bind(&InesctecMrdtHangfaDiscoveryQ2DriverROS2::subMotRef, this,
                  std::placeholders::_1));

    rob_.setSerialPortName(serial_port_name_);
    rob_.openSerial();

    rob_.run = std::bind(&InesctecMrdtHangfaDiscoveryQ2DriverROS2::run, this);
    rob_.init();

    serial_port_timer_ = this->create_wall_timer(
        1s, std::bind(&InesctecMrdtHangfaDiscoveryQ2DriverROS2::checkSerialComms, this));
  }

  void InesctecMrdtHangfaDiscoveryQ2DriverROS2::getParam()
  {

    encoder_res_ = this->declare_parameter<double>("encoder_res", 48.0);
    gear_reduction_ = this->declare_parameter<double>("gear_reduction", 64.0);
    serial_port_name_ =
        this->declare_parameter<std::string>("serial_port_name", "/dev/ttyACM0");

    for (auto &m : rob_.mot)
    {

      m.encoder_res = encoder_res_;

      m.gear_reduction = gear_reduction_;
    }

    RCLCPP_INFO(this->get_logger(),
                "Encoder resolution: %lf (ticks/rev)", rob_.mot[0].encoder_res);

    RCLCPP_INFO(this->get_logger(),
                "Gear reduction ratio: %lf (n:1)", rob_.mot[0].gear_reduction);

    RCLCPP_INFO(this->get_logger(),
                "Serial port: %s", serial_port_name_.c_str());
  }

  void InesctecMrdtHangfaDiscoveryQ2DriverROS2::checkSerialComms()
  {

    if (!rob_.isSerialOpen())
    {
      if (serial_comms_first_fault_)
      {
        serial_comms_first_fault_ = false;
        RCLCPP_INFO(this->get_logger(),
                    "Couldn't open the serial port %s. Will retry every second.",
                    serial_port_name_.c_str());
      }

      rob_.mtx_.lock();
      rob_.stopMotors();
      rob_.mtx_.unlock();

      rob_.closeSerial();
      rob_.openSerial();

      if (rob_.isSerialOpen())
      {
        serial_comms_first_fault_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Opened serial port %s.", serial_port_name_.c_str());

        rob_.init();
      }
    }
  }

  void InesctecMrdtHangfaDiscoveryQ2DriverROS2::run()
  {

    try
    {
      if (rclcpp::Duration(this->now() - sample_time_) >
          rclcpp::Duration::from_seconds(kWatchdogMotWRef))
      {
        rob_.mtx_.lock();
        rob_.stopMotors();
        rob_.mtx_.unlock();
      }
    }
    catch (std::exception &e)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Not possible to check the driver timeout condition (%s)",
                  e.what());

      sample_time_ = this->now();

      return;
    }

    pubMotEnc();
  }

  void InesctecMrdtHangfaDiscoveryQ2DriverROS2::pubMotEnc()
  {

    inesctec_mrdt_drivers_interfaces::msg::MotEncArray msg;

    msg.stamp = this->now();
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

    pub_mot_enc_->publish(msg);
  }

  void InesctecMrdtHangfaDiscoveryQ2DriverROS2::subMotRef(
      const inesctec_mrdt_drivers_interfaces::msg::MotRefArray::SharedPtr msg)
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

} // namespace inesctec_mrdt_hangfa_discovery_q2_driver
