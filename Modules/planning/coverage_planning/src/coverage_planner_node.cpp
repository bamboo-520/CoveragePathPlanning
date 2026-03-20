#include <ros/ros.h>

#include "coverage_planner.h"

using namespace Coverage_Planning;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coverage_planner");

  ros::NodeHandle nh("~");

  Coverage_Planner coverage_planner;
  coverage_planner.init(nh);

  ros::spin();

  return 0;
}
