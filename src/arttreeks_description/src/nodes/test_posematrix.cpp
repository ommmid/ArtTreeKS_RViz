#include <ros/ros.h>
#include <arttreeks_description/posematrix.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_HM");
  ros::NodeHandle nh;

  PoseMatrix pm({1,0,0},{0,1,0},{0,0,0});

  for(int i = 0; i < 3; ++i){
    std::cout << pm.getX()[i] << std::endl;
  }

  std::vector<double> vec1 = {5,6,8,7};
  std::vector<double> vec2 = {8,5,6,2};
  std::vector<double> vec3 = {5,6,7,1};
  PoseMatrix pn(vec1, vec2, vec3);

}
