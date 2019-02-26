#include <ros/ros.h>
#include <arttreeks_description/pluckercoordinate.h>
#include <vector>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_plucker");
  ros::NodeHandle nh;

  std::vector<float> vec1 = {1,2,3};
  std::vector<float> vec2 = {2.5, 4, 3.1};
  PluckerCoordinate<float> line1(vec1, vec2);

  std::vector<float> perp = line1.getPerpPoint();

  std::cout << perp.size() << std::endl;
  std::cout << perp[0] << " " << perp[1] << " " << perp[2] << std::endl;

}
