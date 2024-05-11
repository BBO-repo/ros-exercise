
#include "CovEstNode.h"

int main (int argc, char ** argv)
{
  ros::init (argc, argv, "cov_est");
  
  ros::NodeHandle nh;
  CovEstNode CovEstNode{nh};

  ros::spin();

  return 0;
}
