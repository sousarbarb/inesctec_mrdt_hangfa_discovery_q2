#include "inesctec_mrdt_omnijoy_driver/OmniJoyDriverROS1.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "inesctec_mrdt_omnijoy_driver");

  inesctec_mrdt_omnijoy_driver::OmniJoyDriverROS1 node;

  ros::spin();
  ros::shutdown();

  return 0;
}
