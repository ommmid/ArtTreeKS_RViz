#include "arttreeks_description/robot.h"
#include "arttreeks_description/gnuplot.h"
#include "arttreeks_description/node.h"
#include <vector>
#include <fstream>
#include <iostream>
#include <cmath>
#include <sstream>

Robot::Robot(){

}

Robot::Robot(const std::string &flName, const int &nPos, const int &nPal, const int &numBr,
           const std::vector<int> &pArray, const std::vector<int> &jArray):
  fileName(flName), numPoses(nPos), numPalms(nPal), numBranches(numBr), pA(pArray), jA(jArray)
{
  // get the file content as a string called fileString
  fileString = Robot::getString(fileName);
  //std::cout << std::endl << fileString << std::endl;
  // get joint axes
}

std::string Robot::getString(const std::string &inFile){
  std::string line;
  std::stringstream ss;
  std::ifstream inputFile;
  inputFile.open(inFile);
  if(inputFile.is_open()){
   /* while (getline(inputFile, line)) {
      std::cout << line << std::endl;
    }*/
    ss << inputFile.rdbuf();
    inputFile.close();
  }else{
    std::cout << "unable to open the file" << std::endl;
  }

  return ss.str();
}

std::string Robot::getFileContent(){

  return Robot::fileString;
}


void Robot::createURDF(){

}

