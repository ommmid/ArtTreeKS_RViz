#include <ros/ros.h>
#include <arttreeks_description/edge.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_edge");
  ros::NodeHandle nh;

  // making an edge object
  int numJ = 3;
  std::vector<PluckerCoordinate<double>> plVec;
  std::vector<double> vec11 = {6,8,4}; std::vector<double> vec12 = {1,5,6};
  PluckerCoordinate<double> line1(vec11, vec12);
  plVec.push_back(line1);
  std::vector<double> vec21 = {3,5,4}; std::vector<double> vec22 = {2,5,7};
  PluckerCoordinate<double> line2(vec21, vec22);
  plVec.push_back(line2);
  std::vector<std::vector<double>> jVals;
  std::vector<double> j1Values = {4,5,6}; jVals.push_back(j1Values);
  std::vector<double> j2Values = {1,8,3}; jVals.push_back(j2Values);


  edge edge1(numJ, plVec, jVals);
  std::cout << "edge: number of joint:" << edge1.getNumberOFJoints() << std::endl;
  // access to its elements
  std::vector<PluckerCoordinate<double>> edge1Axes = edge1.getJointAxesInEdge();
  PluckerCoordinate<double> pl = edge1Axes[0];
  std::vector<double> plDirection = pl.getDirection();
  std::cout << "edge: vector:" << plDirection[0] << std::endl;

}
