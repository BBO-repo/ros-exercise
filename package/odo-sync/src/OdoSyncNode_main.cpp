
#include "OdoSyncNode.h"

int main (int argc, char ** argv)
{
  ros::init (argc, argv, "odo_sync");
  
  ros::NodeHandle nh;
  OdoSyncNode odoSyncNode{nh};

  ros::spin();

  return 0;
}
