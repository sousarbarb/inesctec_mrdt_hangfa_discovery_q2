#include "inesctec_mrdt_localization_odom/OdomWhROS1.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "inesctec_mrdt_localization_odom");

  inesctec_mrdt_localization_odom::OdomWhROS1 node;

  ros::spin();
  ros::shutdown();

  return 0;
}
