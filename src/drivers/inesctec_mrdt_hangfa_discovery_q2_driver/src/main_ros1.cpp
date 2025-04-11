#include "inesctec_mrdt_hangfa_discovery_q2_driver/InesctecMrdtHangfaDiscoveryQ2DriverROS1.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "inesctec_mrdt_hangfa_discovery_q2_driver");

  inesctec_mrdt_hangfa_discovery_q2_driver::
      InesctecMrdtHangfaDiscoveryQ2DriverROS1 node;

  ros::spin();
  ros::shutdown();

  return 0;
}
