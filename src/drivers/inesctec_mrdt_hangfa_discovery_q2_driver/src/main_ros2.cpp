#include "inesctec_mrdt_hangfa_discovery_q2_driver/InesctecMrdtHangfaDiscoveryQ2DriverROS2.h"

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<inesctec_mrdt_hangfa_discovery_q2_driver::InesctecMrdtHangfaDiscoveryQ2DriverROS2>());

  rclcpp::shutdown();

  return 0;
}
