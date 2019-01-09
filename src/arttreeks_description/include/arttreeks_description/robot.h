#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <vector>
#include <fstream>
#include <arttreeks_description/node.h>

class Robot
{
public:
  int numPoses;
  int numPalms;
  int numBranches;

  std::string fileName; // including directory
  std::vector<int> pA;
  std::vector<int> jA;
  // constructor
  Robot();
  Robot(const std::string &flName, const int &nPos, const int &nPal, const int &numBr,
        const std::vector<int> &pArray, const std::vector<int> &jArray);

  std::string getFileContent();
  // functions
  // plot the parentArray
  void createURDF();

private:
  std::string fileString;
  std::string getString(const std::string &inFile);
};

#endif // ROBOT_H
