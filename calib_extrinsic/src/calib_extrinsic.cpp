#include "ros/ros.h"
#include "calibratebasecamera.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ImgRecevier");
  ros::NodeHandle n;
  std::string configFileName;
//   if (!ros::param::get("~config_file", configFileName))
//   {
//     ROS_ERROR("IbvsConstrained: no config file name given, exit!");
//     exit(1);
//   }
  configFileName = argv[1];
  calibrateBaseCamera mCalib(n,configFileName);
  ros::spin();
  return 0;
}

