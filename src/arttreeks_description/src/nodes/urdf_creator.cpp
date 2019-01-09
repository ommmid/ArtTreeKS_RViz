#include <ros/ros.h>
#include <fstream>
#include <arttreeks_description/robot.h>
#include <vector>
#include <stdio.h>
#include "arttreeks_description/gnuplot.h"

// the input of this program is the output of the ArtTreeKS and the output is an URDF in urdf_creator_output.cpp

class Test {
    int &t;
public:
    Test (int &x) : t(x) {}

};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "urdf_creator");
  ros::NodeHandle nh;

  // ------------------------------------------------
  // read the output of ArtTreeKS and extract the joints direction and moment in reference configuration
  std::vector<int> vec1 = {0,1,1,1};
  std::vector<int> vec2 = {2,4,4,4};
  int numPose = 5;
  int numPalms = 3;
  int numBranches = 4;
  std::string fileName = "ArtTreeKS_Output.txt";

  Robot rob(fileName, numPose, numPalms, numBranches, vec1, vec2);
  std::string str = rob.getFileContent();
  std::cout << str << std::endl;

  // ------------------------------------------------
  // write to xacro file
  std::ofstream fw;
  fw.open("/home/omid/ros_ws/arttreeks_rviz/src/arttreeks_description/urdf/creator_output.xacro");
  if(fw.is_open()){
    std::cout << "Successful open of file" << std::endl;
    fw << "<?xml version=\"1.0\" ?> \n";
    fw << "<robot name=\"atks\" xmlns:xacro=\"http://ros.org/wiki/xacro\"> \n";

    fw << "</robot>";
  }
  fw.close();

  std::cin.get();
}



