#include "inesctec_mrdt_localization_odom/OdomWhROS2.h"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inesctec_mrdt_localization_odom::OdomWhROS2>());
  rclcpp::shutdown();

  return 0;
}
