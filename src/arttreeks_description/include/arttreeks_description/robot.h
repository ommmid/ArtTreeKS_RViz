#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <vector>
#include <fstream>
#include <arttreeks_description/node.h>
#include <arttreeks_description/pluckercoordinate.h>
#include <arttreeks_description/edge.h>
#include <arttreeks_description/posematrix.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


class Robot
{
public:
  int numPoses;
  std::vector<int> branchesEE; // edge numbers that are at the end of each branch
  std::vector<int> eeIndexes; // we need the indexes of ee in the above series
  int numBranches;
  std::vector<int> branches; // all the branches in series defined by the edge numbers

  std::string outputFileName; // including directory
  std::string inputFileName; // including directory
  std::vector<int> pA;
  std::vector<int> jA;
  std::vector<std::vector<double>> jointLinkStartPoint;

  // constructor
  Robot();
  Robot(const std::string &inFName, const std::string &outFName);

  std::string getInputFileContent();
  std::string getOutputFileContent();
  void plotTree();
  void createURDF(std::string xacroFile, const double &linkz_thikness, const double linkx_thikness);
  std::vector<edge> getEdgeArray();
  std::vector<PluckerCoordinate<double>> getJointAxesArray();

private:
  std::string inputFileString; // the content of ArtTreeKS output in string format
  std::string outputFileString; // the content of ArtTreeKS output in string format
  void getString();
  std::vector<int> findChildrenByParentIndex(const std::vector<int> &pArr, const int &parentIndex);
  std::vector<int> findChildrenByParentValue(const std::vector<int> &pArr, const int &parentValue);
  bool isValInArray(const std::vector<int> &pArry, const int &val);
  std::vector<edge> edgeArray;
//std::vector<int> testVector;
  std::vector<PluckerCoordinate<double>> jointAxesArray;
  std::vector<PluckerCoordinate<double>> xAxesArray;
  std::vector<std::vector<double>> jointValuesArray;
  std::vector<std::vector<PoseMatrix>> eePoseMatrix; // each branch has a list of poses
  std::vector<std::vector<tf2::Quaternion>> eePoseQuaternion;
  void extractJointInformation(); // extract the joint axes from fileString and set them to edgeArray
  int materialCounter;

  /*
   * @brief extract the poses of EE in Matrix for each branch. All the branches have
   *  same number of poses for EE. eePoseMatrix = {{branch1_pose1, branch1_pose2 ...}, {branch2_pose1}, {} ....}
   *
   * */
  void extractEEPoseMatrix(); //
  void extractEEPoseQuaternion();

  // urdf functions
  // pose is a 6-element vector, three for position and three for orientation
  std::string fixedJoint(std::string name, std::string parent, std::string child, std::vector<double> pose);
  std::string revoluteJoint(std::string name, std::string parent, std::string child, std::vector<double> pose, std::vector<double> axis);
  std::string boxLink(std::string name, std::vector<double> pose, std::vector<double> size);
  std::string cylinderLink(std::string name, std::vector<double> pose, std::vector<double> size, std::vector<double> rgba);
  // the direction of the cylinder is in z-axis

  std::vector<double> crossProduct(const std::vector<double> &v1, const std::vector<double> &v2);
  double dotProduct(const std::vector<double> &v1, const std::vector<double> &v2);
  std::vector<double> sumVector(const std::vector<double> &v1, const std::vector<double> &v2);
  double vectorMagnitude(const std::vector<double> &v);
  std::vector<double> vectorMultipliedByDouble(const std::vector<double> &v, const double &d);
  std::vector<double> vectorDividedByDouble(const std::vector<double> &v, const double &d);
  void calculateCommonNormal(const PluckerCoordinate<double> &pl1, const PluckerCoordinate<double> &pl2,
                             PluckerCoordinate<double> *pl3, double *distance, double *angle,
                             std::vector<double> &cl1, std::vector<double> &cl2, std::vector<double> &arbP);




};
#endif // ROBOT_H
