#include <ros/ros.h>

// the input of this program is the output of the ArtTreeKS and the output is an URDF in urdf_creator_output.cpp

int main(int argc, char **argv)
{
  ros::init(argc, argv, "urdf_creator");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
