#include "arttreeks_description/robot.h"
#include "arttreeks_description/gnuplot.h"
#include "arttreeks_description/node.h"
#include "arttreeks_description/edge.h"
#include "arttreeks_description/posematrix.h"
#include <arttreeks_description/funcs.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <tf2/LinearMath/Matrix3x3.h>

const double PI = 3.14159265359;

Robot::Robot(){

}

Robot::Robot(const std::string &inFName, const std::string &outFName):
inputFileName(inFName),outputFileName(outFName) //, numPoses(nPos), edgeArray(jA.size(), edge())
{ //edgeArray(jA.size(), edge())
  // get the file content as a string called fileString
  Robot::getString();

  //set pA and jA
  std::size_t p_start_index = inputFileString.find("parentPointer = {", 0) + 17;
  std::cout << "p_start_index " << p_start_index << std::endl;
  std::size_t p_end_index = inputFileString.find("}" , p_start_index);
  std::cout << "p_end_index " << p_end_index << std::endl;
  std::string subPP = inputFileString.substr(p_start_index, p_end_index - p_start_index);
  std::cout << "subPP ===>>>" << subPP << std::endl;
  std::vector<double> PP= funcs::convertString2Vector(subPP);
  for(auto m : PP){
    pA.push_back(m);
  }
  funcs::coutVector("pA " , pA);
  std::size_t j_start_index = inputFileString.find("edges = {", 0) + 9;
  std::size_t j_end_index = inputFileString.find("}" , j_start_index);
  std::string subJA = inputFileString.substr(j_start_index, j_end_index - j_start_index);
  std::cout << "subJA ===>>>" << subJA << std::endl;
  std::vector<double> JA = funcs::convertString2Vector(subJA);
  for(auto m : JA){
    jA.push_back(m);
  }
  funcs::coutVector("jA", jA);
  std::size_t mp_start_index = inputFileString.find("mp = ", 0) + 4;
  std::size_t mp_end_index = inputFileString.find("mv = " , mp_start_index) - 1;
  std::string subMP = inputFileString.substr(mp_start_index, mp_end_index - mp_start_index);
  std::cout << "subMP ===>>>" << subMP << std::endl;
  std::istringstream oss(subMP);
  oss >> numPoses;
  std::cout << "numPoses: " << numPoses << std::endl;

  // get the branches
  int px = pA.size();
  for(int u = 1; u <= px; ++u){
    std::vector<int>::iterator it = std::find(pA.begin(), pA.end(), u);
    if(it == pA.end()){ // if true, then u is NOT in the pA and that is the end of the branch, ee
      branchesEE.push_back(u);
    }
  }
  numBranches = branchesEE.size();

  // make an object of class Node. branches is going to have the edge numbers in order to reach the
  // bracnhes
  branches.push_back(0);
  Node tree(0, pA, branches);
  for(auto item : branches){
    std::cout << "branches: " << item << std::endl;
  }


 //std::vector<edge> edgeArr(jA.size(), edgeStart);
  extractJointInformation();

  for(int h= 0; h <jointAxesArray.size(); ++h){
    PluckerCoordinate<double> pluckerTemp = jointAxesArray[h];
    std::vector<double> pDir = pluckerTemp.getDirection();
    std::cout << " plucker direction: " << pDir[0] << " " << pDir[1] << " " << pDir[2] << std::endl;
    std::vector<double> pMom = pluckerTemp.getMoment();
    std::cout << " plucker moment " << pMom[0] << " " << pMom[1] << " " << pMom[2] << std::endl;
    std::cout << "---------------------" << std::endl;
  }/*
  for(int h= 0; h <jointValuesArray.size(); ++h){
    std::vector<double> vecT = jointValuesArray[h];
    std::string stdVec;
    for(int g = 0; g < vecT.size(); ++g){
      stdVec = stdVec + " " + std::to_string( vecT[g] ) + " ";
    }
    std::cout << stdVec << std::endl;
    std::cout << "---------------------" << std::endl;
  }*/

  // test calculateCommonNormal();
/*  std::vector<double> dir1V ={0,0,1}; std::vector<double> dir1 = vectorDividedByDouble(dir1V, 5);
  std::vector<double> pLine1 = {0,0,0}; std::vector<double> mom1 = crossProduct(pLine1, dir1);
  PluckerCoordinate<double> plk1(dir1, mom1);
  std::vector<double> dir2V ={0,1,0}; std::vector<double> dir2 = vectorDividedByDouble(dir2V, 5);
  std::vector<double> pLine2 = {2,0,2}; std::vector<double> mom2 = crossProduct(pLine2, dir2);
  PluckerCoordinate<double> plk2(dir2, mom2);
  PluckerCoordinate<double> *plk3 = new PluckerCoordinate<double>();
  double *dist = new double(); double *ang = new double();
  std::vector<double> cl1; std::vector<double> cl2;
  calculateCommonNormal(plk1, plk2, plk3, dist, ang, cl1, cl2, pLine1);
 // std::cout << "distance " << *dist << std::endl;
 // std::cout << "angle " << *ang << std::endl;
  funcs::coutVector("cl1 => ", cl1);
  funcs::coutVector("cl2 => ", cl2);
*/

  extractEEPoseMatrix();
  extractEEPoseDualQuaternion();

  plotTree();
  materialCounter =0;
}

void Robot::getString(){
  std::stringstream out_ss;
  std::ifstream outputFile;
  outputFile.open(outputFileName);
  if(outputFile.is_open()){
    out_ss << outputFile.rdbuf();
    outputFile.close();
  }else{
    std::cout << "unable to open output file" << std::endl;
  }
  outputFileString = out_ss.str();

  std::stringstream in_ss;
  std::ifstream inputFile;
  inputFile.open(inputFileName);
  if(inputFile.is_open()){
    in_ss << inputFile.rdbuf();
    inputFile.close();
  }else{
    std::cout << "unable to open output file" << std::endl;
  }
  inputFileString = in_ss.str();
}

std::string Robot::getInputFileContent(){
  return inputFileString;
}

std::string Robot::getOutputFileContent(){
  return outputFileString;
}

void Robot::plotTree(){
  int tcpLevel = pA.back() + 1;
  // create a vector with size pA.back that has vectors as its elemets. each of thes elements is
  // another vector that has two elements of zero
  std::vector<std::vector<float>> coordinateArray(pA.size(), std::vector<float>(2,0));
  int xCounter = 0;
  for(int i = pA.size()-1; i >= 0; --i){
    std::vector<float> temp;
    if(!isValInArray(pA, i+1)){ // creat tcp points
      std::cout << "----??????----- " << i << std::endl;
      temp.push_back(xCounter); // x
      temp.push_back(tcpLevel); // y
      xCounter += 2;
      std::cout << "----counterrrr----- " << xCounter << std::endl;

    }else { // for non-tcp points, look at the children and calculate the mean in x-direction
      // find the index in pA of children.
      std::vector<int> indVec = findChildrenByParentIndex(pA, i);
      // find the minimum
      float sum = 0;
      for(int j = 0; j < indVec.size(); ++j){
        sum = coordinateArray[indVec[j]][0] + sum;
        std::cout << "----HHHHH----- " << indVec[j] << "summmm " << sum << std::endl;

      }
      std::cout << "-------------- " << indVec.size() << std::endl;
      float mean = sum / (float)indVec.size();
      temp.push_back(mean); // x
      temp.push_back(i+1); // y
    }
    coordinateArray[i] = temp;
  }
  //std::reverse(coordinateArray.begin(), coordinateArray.end());
  for(int q = 0; q < coordinateArray.size(); ++q){
      std::cout <<  coordinateArray[q][0] << " " << coordinateArray[q][1] << std::endl;
  }
  // handle node 0
  std::vector<float> coordinate0;
  int val = 0;
  std::vector<int> zeroVec = findChildrenByParentValue(pA, val);
  float sum = 0;
  for(int s = 0; s < zeroVec.size(); ++s){
  std::cout << "zzzzzzzzzzzzzz " <<  std::to_string( zeroVec[s]) << std::endl;
    sum = coordinateArray[zeroVec[s]][0] + sum;
  }
  float mean = sum / zeroVec.size();
  coordinate0.push_back(mean); // x
  coordinate0.push_back(0); // y

  // now start from the end of pArr and for each element make lines till it reaches to the parent
  // step1: one level down. step2: y const and x reches to parent's x
  // child
  //   |
  //   |
  //   *----parent

  std::ofstream ft;
  ft.open("points.dat");
  if(ft.is_open()){

    for(int i = pA.size()-1; i >= 0; --i){

      if(pA[i] != 0){
        int pIndex = pA[i]-1; // index is one less than edge number
        ft << std::to_string(coordinateArray[i][0]) << " " << std::to_string(coordinateArray[i][1])
            << " " << std::to_string(i+1) << " " << std::to_string(coordinateArray[i][0]) << " " << std::to_string(coordinateArray[i][1]) << "\n";
        ft << std::to_string(coordinateArray[i][0]) << " " << std::to_string(coordinateArray[pIndex][1]) << "\n \n";

        ft << std::to_string(coordinateArray[i][0]) << " " << std::to_string(coordinateArray[pIndex][1]) << "\n";
        ft << std::to_string(coordinateArray[pIndex][0]) << " " << std::to_string(coordinateArray[pIndex][1]) << "\n \n";
      }else{ // if the element in pA is equal zero
        ft << std::to_string(coordinateArray[i][0]) << " " << std::to_string(coordinateArray[i][1])
            << " " << std::to_string(i+1) << " " << std::to_string(coordinateArray[i][0]) << " " << std::to_string(coordinateArray[i][1]) << "\n";
        ft << std::to_string(coordinateArray[i][0]) << " " << std::to_string(coordinate0[1]) << "\n \n";

        ft << std::to_string(coordinateArray[i][0]) << " " << std::to_string(coordinate0[1]) << "\n";
        ft << std::to_string(coordinate0[0]) << " " << std::to_string(coordinate0[1]) << "\n \n";
      }
    }

    ft << std::to_string(coordinate0[0]) << " " << std::to_string(coordinate0[1])
        << " " << std::to_string(0) << " " << std::to_string(coordinate0[0]) << " " << std::to_string(coordinate0[1]) << "\n";

  }
  ft.close();

  xLevelCounter = xCounter;
  tcpLevelCounter = tcpLevel;
}

// given parentArray and an index, this functions find the indexes of the children of that index
std::vector<int> Robot::findChildrenByParentIndex(const std::vector<int> &pArr,const int &parentIndex){
  // point: edge number in parentArray is one more than index
  // pA = {0,1,1,1}, edgeNumber = {1,2,3,4}. So edge1 has threee children that are 2,3 and 4

  std::vector<int> childrenIndex;
  int edgeNumber = parentIndex + 1;
  for(int i = 0; i < pArr.size(); ++i){
    if(pArr[i] == edgeNumber){
      childrenIndex.push_back(i);
    }
  }

  return childrenIndex;
}
// find based on parent value not parent index
std::vector<int> Robot::findChildrenByParentValue(const std::vector<int> &pArr, const int &parentValue){
  std::vector<int> childrenIndex;
  for(int i = 0; i < pArr.size(); ++i){
    if(pArr[i] == parentValue){
      childrenIndex.push_back(i);
    }
  }

  return childrenIndex;
}

bool Robot::isValInArray(const std::vector<int> &pArry, const int &val){
  for(int r = 0; r < pArry.size(); ++r){
    if(val == pArry[r]){
      return true;
    }
  }
  return false;
}

void Robot::extractJointInformation(){
// extract the joint axes from fileString and save it to a vector whose type is pluckercoordinate
  // for joint axes and another one with type std::vector<double> for joint values

  int sumJoint = 0;
  for(int z = 0; z < jA.size(); ++z){
    sumJoint = jA[z] + sumJoint;
  }

  std::remove("jointsPluckerCoordinates.txt");
  int startingSearchIndex = 0;
  for(int p = 0; p < sumJoint; ++p){

  // find the index of the first letter of the first instance of "setPositions( {"
  std::size_t position_start_index = outputFileString.find("setPositions( {", startingSearchIndex) + 15; // start index of the search is the
  if(position_start_index == -1){ std::cout << "check ArtTreeKS output file" << std::endl;  }
  // second argument which is 0 for the first search
  std::size_t position_end_index = outputFileString.find("}" , position_start_index);
  std::string subP = outputFileString.substr(position_start_index, position_end_index - position_start_index); // (starting index, size of the substring)
 // std::cout << "sub ===> " << subP << std::endl;

  std::vector<double> setPoistions = funcs::convertString2Vector(subP);
  //std::cout << "setPoistions seizeeeee "<< setPoistions.size() << std::endl;
  std::cout.precision(17);
  for(int u = 0; u < setPoistions.size(); ++u){
    //std::cout << "u: " << setPoistions[u] << std::endl;
  }
/*
 // set edgeArray
  std::vector<std::vector<double>> jValues;
  jValues.push_back(setPoistions);
  std::vector<PluckerCoordinate<double>> axesArray;
  edge edgeTemp(jA[0], axesArray , jValues);
  edgeArray[2] = edgeTemp;
*/

  // based on the depth-order of the parentArray we push the plucker coordinate extracted from
  // fileString to jointAxesArray and jointValuesArray

  jointValuesArray.push_back(setPoistions);

  std::size_t plucker_direction_start_index = outputFileString.find("setPlucker( {", startingSearchIndex) + 13; // start index of the search is the
  std::size_t plucker_direction_end_index = outputFileString.find("}" , plucker_direction_start_index + 1);
  std::string subPD = outputFileString.substr(plucker_direction_start_index, plucker_direction_end_index - plucker_direction_start_index);
  std::vector<double> pluckerDir = funcs::convertString2Vector(subPD);

  std::size_t plucker_moment_start_index = plucker_direction_end_index + 4; // start index of the search is the
  std::size_t plucker_moment_end_index = outputFileString.find("}" , plucker_moment_start_index + 1);
  std::string subPM = outputFileString.substr(plucker_moment_start_index, plucker_moment_end_index - plucker_moment_start_index);
  std::vector<double> pluckerMom = funcs::convertString2Vector(subPM);


  PluckerCoordinate<double> plucker(pluckerDir, pluckerMom);
  jointAxesArray.push_back(plucker);
  startingSearchIndex = plucker_moment_end_index + 10;

  // write the joint values in a file so I can read it from another node to publish them for rviz
  std::ofstream fm;
  fm.open("jointsPluckerCoordinates.txt", std::ios_base::app);
  if(fm.is_open()){
    fm << std::to_string(pluckerDir[0]) << ", " << std::to_string(pluckerDir[1]) << ", " <<
          std::to_string(pluckerDir[2]) << ", " << std::to_string(pluckerMom[0]) << ", " <<
          std::to_string(pluckerMom[1]) << ", " << std::to_string(pluckerMom[2]) << "\n";
  }
  fm.close();

/*
  std::cout << "position_start_index " << position_start_index << std::endl;
  std::cout << "position_end_index " << position_end_index << std::endl;
  std::cout << "plucker_direction_start_index: " << plucker_direction_start_index << std::endl;
  std::cout << "plucker_direction_end_index: " << plucker_direction_end_index << std::endl;
  std::cout << "plucker_moment_start_index: " << plucker_moment_start_index << std::endl;
  std::cout << "plucker_moment_end_index: " << plucker_moment_end_index << std::endl;
  std::cout << "------------------------------" << std::endl; */
  }


}

std::vector<PluckerCoordinate<double>> Robot::getJointAxesArray(){

  return jointAxesArray;
}

void Robot::extractEEPoseMatrix(){
  // create eePoseMatrix

std::cout << "numberofBranches " << numBranches << std::endl;
  int startingTCPIndex =0;
  for(int r = 0; r < numBranches; ++r){
    std::size_t tcp_start_index = inputFileString.find("tcp", startingTCPIndex) + 10;
    int startingFrameIndex = tcp_start_index;
    std::vector<PoseMatrix> vecPoseM;
    for(int k = 0; k < numPoses; ++k){

      std::size_t frist_row_start_index = outputFileString.find("{ { ", startingFrameIndex) + 3;
      std::size_t first_row_end_index = outputFileString.find("}", frist_row_start_index) - 1;
      std::string subFirst = outputFileString.substr(frist_row_start_index, first_row_end_index - frist_row_start_index); // (starting index, size of the substring)
      std::vector<double> firstRow = funcs::convertString2Vector(subFirst);
      std::cout.precision(17);
      std::cout << firstRow[0] << " " << firstRow[1] << " " << firstRow[2] << " " << firstRow[3] << std::endl;

      std::size_t second_row_start_index =  outputFileString.find("{", first_row_end_index) + 1;
      std::size_t second_row_end_index = outputFileString.find("}", second_row_start_index) - 1;
      std::string subSecond = outputFileString.substr(second_row_start_index, second_row_end_index - second_row_start_index);
      std::vector<double> secondRow = funcs::convertString2Vector(subSecond);
      std::cout.precision(17);
      std::cout << secondRow[0] << " " << secondRow[1] << " " << secondRow[2] << " " << secondRow[3] << std::endl;

      std::size_t third_row_start_index = outputFileString.find("{", second_row_end_index) + 1;
      std::size_t third_row_end_index = outputFileString.find("}", third_row_start_index) - 1;
      std::string subThird = outputFileString.substr(third_row_start_index, third_row_end_index - third_row_start_index);
      std::vector<double> thirdRow = funcs::convertString2Vector(subThird);
      std::cout.precision(17);
      std::cout << thirdRow[0] << " " << thirdRow[1] << " " << thirdRow[2] << " " << thirdRow[3] << std::endl;

      startingFrameIndex = third_row_end_index + 5;
      std::cout << "--------------------" << std::endl;

      PoseMatrix pmt(firstRow, secondRow, thirdRow);
      vecPoseM.push_back(pmt);
    }
    startingTCPIndex = tcp_start_index + 5;
    eePoseMatrix.push_back(vecPoseM);
  }

}


void Robot::extractEEPoseDualQuaternion(){

  for(int r = 1; r <= numBranches; ++r){

    std::vector<std::vector<double>> eePoseQuat;
    for(int p = 1; p <= numPoses; ++p){
      std::string targetString = "P[" + std::to_string(p) + "][" + std::to_string(r) + "]" ;
      std::size_t start_index = inputFileString.find(targetString, 0) + 19;
      std::size_t end_index = inputFileString.find("}", start_index);
      std::string subStr = inputFileString.substr(start_index, end_index - start_index);
      std::vector<double> dq = funcs::convertString2Vector(subStr);
      std::vector<double> dqC = {dq[1], dq[2], dq[3], dq[0], dq[4], dq[5], dq[6], dq[7]};
      eePoseQuat.push_back(dqC);
    }
    eePoseDualQuaternion.push_back(eePoseQuat);
  }
}

std::vector<std::vector<std::vector<double>>> Robot::getEEPoseDualQuaternion(){
  return eePoseDualQuaternion;
}

std::vector<edge> Robot::getEdgeArray(){
  return edgeArray;
}

// urdf functions
std::string Robot::fixedJoint(std::string name, std::string parent, std::string child, std::vector<double> pose){
  std::string outString =
  "  <joint name=\"" + name + "\" type=\"fixed\">  \n"
  "    <parent link=\"" + parent + "\"/>   \n"
  "    <child link=\"" + child + "\"/>  \n"
  "    <origin xyz=\"" + std::to_string(pose[0]) +" "+ std::to_string(pose[1]) +" "+ std::to_string(pose[2])
          +"\" rpy=\"" + std::to_string(pose[3]) +" "+ std::to_string(pose[4]) +" "+ std::to_string(pose[5]) + "\"/>   \n"
  "  </joint>   \n";

  return outString;
}
std::string Robot::revoluteJoint(std::string name, std::string parent, std::string child, std::vector<double> pose, std::vector<double> axis){
  std::string outString =
    "  <joint name=\"" + name + "\" type=\"revolute\"> \n"
    "   <parent link=\""+ parent +"\"/>    \n"
        "    <child link=\"" + child +"\"/>   \n"
   "    <origin xyz=\"" + std::to_string(pose[0]) +" "+ std::to_string(pose[1]) +" "+ std::to_string(pose[2])
           +"\" rpy=\"" + std::to_string(pose[3]) +" "+ std::to_string(pose[4]) +" "+ std::to_string(pose[5]) + "\"/>   \n"
        "   <limit effort=\"1000.0\" lower=\"-3.14\" upper=\"3.14\" velocity=\"0.5\"/>    \n"
   "  <axis xyz=\"" + std::to_string(axis[0]) +" "+ std::to_string(axis[1]) +" "+ std::to_string(axis[2]) + "\"/>   \n"
      "   </joint> \n";

  return outString;
}
std::string Robot::boxLink(std::string name, std::vector<double> pose, std::vector<double> size){
  std::string outString =
    "  <link name=\"" + name + "\">    \n"
        "  <visual>   \n"
        "   <geometry>   \n"
        "     <box size=\"" + std::to_string(std::abs(size[0])) +" "+ std::to_string(std::abs(size[1])) +" "+ std::to_string(std::abs(size[2])) + "\"/>  \n"
        "    </geometry>   \n"
   "    <origin xyz=\"" + std::to_string(pose[0]) +" "+ std::to_string(pose[1]) +" "+ std::to_string(pose[2])
           +"\" rpy=\"" + std::to_string(pose[3]) +" "+ std::to_string(pose[4]) +" "+ std::to_string(pose[5]) + "\"/>   \n"
        "    <material name=\"blue\">   \n"
        "       <color rgba=\"0 0 0.8 1\"/>   \n"
        "   </material>   \n"
       "   </visual>   \n"
      "</link>  \n";

  return outString;
}
std::string Robot::cylinderLink(std::string name, std::vector<double> pose, std::vector<double> size, std::vector<double> rgba){
  ++materialCounter;
  std::string outString =
     "  <link name=\""+ name +"\">   \n"
     "      <visual>   \n"
     "       <geometry>    \n"
     "         <cylinder length=\"" + std::to_string(std::abs(size[0])) +"\" radius=\"" + std::to_string(std::abs(size[1])) +"\"/>    \n"
     "       </geometry>   \n"
    "        <origin xyz=\"" + std::to_string(pose[0]) +" "+ std::to_string(pose[1]) +" "+ std::to_string(pose[2])
    +"\" rpy=\"" + std::to_string(pose[3]) +" "+ std::to_string(pose[4]) +" "+ std::to_string(pose[5]) + "\"/>   \n"
     "       <material name=\"Material_"+ std::to_string(materialCounter) +"\"> \n"
     "         <color rgba=\""+std::to_string(rgba[0]/255) + " " + std::to_string(rgba[1]/255) + " " +std::to_string(rgba[2]/255) + " " +std::to_string(rgba[3]) +"\"/> \n"
     "       </material>  \n"
     "     </visual>   \n"
     "   </link>    \n" ;

  return outString;
}

std::vector<double> Robot::vectorMultipliedByDouble(const std::vector<double>& v, const double& d){
  std::vector<double> outV = {v[0]*d, v[1]*d, v[2]*d};
  return outV;
}
std::vector<double> Robot::vectorDividedByDouble(const std::vector<double>& v, const double &d){
  std::vector<double> outV = {v[0]/d, v[1]/d, v[2]/d};
  return outV;
}
double Robot::vectorMagnitude(const std::vector<double> &v){
  return sqrt( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] );
}
double Robot::dotProduct(const std::vector<double> &v1, const std::vector<double> &v2){
  return v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2];
}
std::vector<double> Robot::crossProduct(const std::vector<double> &v1, const std::vector<double> &v2){
  std::vector<double> cross;
  cross.push_back(v1[1]*v2[2]-v1[2]*v2[1]);
  cross.push_back(v1[2]*v2[0]-v1[0]*v2[2]);
  cross.push_back(v1[0]*v2[1]-v1[1]*v2[0]);
  return cross;
}
std::vector<double> Robot::sumVector(const std::vector<double> &v1, const std::vector<double> &v2){
  std::vector<double> outV = {v1[0]+v2[0], v1[1]+v2[1], v1[2]+v2[2]};
  return outV;
}

void Robot::calculateCommonNormal(const PluckerCoordinate<double> &pl1, const PluckerCoordinate<double> &pl2,
                        PluckerCoordinate<double> *pl3, double *distance, double *angle,
                        std::vector<double> &cl1, std::vector<double> &cl2, std::vector<double> &arbP){
  // pl1 = > line1, pl2 => line2, pl3 => perpendicular line, cl1 => the point that common normal intersetc
  // line 1, cl2 => the point that common normal intersect line2, arbP => in case that two lines are
  // parallel, we need an arbitraty point on the first line (I do not want the perpendicular point to the origin)
  // I want arbP to be the origin of the previous frame, jointLinkStartPoint of the previous frame

  std::cout << "*******************" << std::endl;
  std::vector<double> d12 = crossProduct(pl1.getDirection(), pl2.getDirection());
  double d12Mag = vectorMagnitude(d12);

  double dtP = dotProduct(pl1.getDirection(), pl2.getDirection());
  if(d12Mag < 0.001){ // two lines are parallel
    std::vector<double> perpPointOnLine2 = crossProduct(pl2.getDirection(), pl2.getMoment());
    std::vector<double> wVector = sumVector(perpPointOnLine2, vectorMultipliedByDouble( arbP, -1 ) );
    std::vector<double> uVector = crossProduct(pl1.getDirection(), wVector);
    *distance = vectorMagnitude(uVector);
    cl1 = arbP;
    std::vector<double> uVpl1 = crossProduct(uVector, pl1.getDirection());
    double uVpl1_mag = vectorMagnitude(uVpl1);
    std::vector<double> normalDirection = vectorDividedByDouble( uVpl1 , uVpl1_mag);
    std::vector<double> normalVector = vectorMultipliedByDouble(normalDirection, *distance);
    cl2 = sumVector(cl1, normalVector);
    std::vector<double> normalMoment = crossProduct(arbP, normalDirection);
    *pl3 = PluckerCoordinate<double> (normalDirection, normalMoment);
    if(dtP > 0){ // same direction
      *angle = 0;
    }else{ // oposite direction
      *angle = PI;
    }
  }else{ // two lines are not parallel
    std::vector<double> d3 = vectorDividedByDouble(d12, d12Mag);
    funcs::coutVector("d12 ==> ", d12);

    std::vector<double> m12 = sumVector(crossProduct(pl1.getDirection(), pl2.getMoment()), crossProduct(pl1.getMoment(), pl2.getDirection()));
    std::vector<double> m3 = vectorDividedByDouble(crossProduct(crossProduct(d12, m12), d12) , d12Mag*d12Mag*d12Mag);

    // cross product follows the right hand rule considering the smaller angle between lines
    // for example it considers -30 instead of 330, then in this case the common normal direction is
    // downwards, not from first line to the second line
    // but what we want is to have the direction of the common normal ALWAYS from first line to the next line
    // regardless of the angle between them.
    // if the dot product of the vector connecting the normal points on the line from first to second
    // with the cross product of the lines is positive, then the cross product is from frist to second
    // otherwise, we need to reverse its direction
    std::vector<double> p1 = crossProduct(pl1.getDirection(), pl1.getMoment());
    std::vector<double> p2 = crossProduct(pl2.getDirection(), pl2.getMoment());
    double aligned = dotProduct( sumVector(p2, vectorMultipliedByDouble(p1, -1)), d12);
    // if aligned > 0 => cross product of two lines is directing from first one to the second one
    // if aligned < 0 => cross product of two lines is directing from second one to the first one
    // aligned can NOT be zero
    if(aligned < 0){
      d3 = vectorMultipliedByDouble(d3, -1);
      m3 = vectorMultipliedByDouble(m3, -1);
    }
    PluckerCoordinate<double> pluk3(d3, m3);
    *pl3 = pluk3;

    std::vector<double> c1 = sumVector(crossProduct(d3, m3) ,vectorMultipliedByDouble(d3, dotProduct( p1, d3)) );
    std::vector<double> c2 = sumVector(crossProduct(d3, m3) ,vectorMultipliedByDouble(d3, dotProduct( p2, d3)) );
    cl1 = c1;
    cl2 = c2;

    std::vector<double> c12 = sumVector(c2, vectorMultipliedByDouble(c1, -1));
    *distance = vectorMagnitude( c12 );

    double cosTheta = dotProduct(pl1.getDirection(), pl2.getDirection());
    if(aligned > 0){ // same direction
      *angle = acos(cosTheta);
    }else if(aligned < 0){ // opposite direction
      *angle = - acos(cosTheta);
    }

  }

  funcs::coutPlucker("pl1 >>>>> " , pl1);
  funcs::coutPlucker("pl2 >>>>> " , pl2);
  std::cout << "distance: " << *distance << "            angle: " << *angle << std::endl;
}

void Robot::createURDF(std::string xacroFile, const double &linkz_thikness, const double linkx_thikness){
  std::ofstream fw;
  fw.open(xacroFile);
  if(fw.is_open()){
    std::cout << "Successfully opened the file" << std::endl;
    fw << "<?xml version=\"1.0\" ?> \n";
    fw << "<robot name=\"atks\" xmlns:xacro=\"http://ros.org/wiki/xacro\"> \n";
    fw << " <link name=\"base\"/> \n";

    PluckerCoordinate<double> refZ({0,0,1},{0,0,0});
    PluckerCoordinate<double> *pluckerPreviousZ = new PluckerCoordinate<double>(); *pluckerPreviousZ =refZ;
    PluckerCoordinate<double> *pluckerNextZ = new PluckerCoordinate<double>();
    PluckerCoordinate<double> *pluckerTempZ = new PluckerCoordinate<double>();
    PluckerCoordinate<double> refX({1,0,0},{0,0,0});
    PluckerCoordinate<double> *pluckerPreviousX = new PluckerCoordinate<double>(); *pluckerPreviousX =refX;
    PluckerCoordinate<double> *pluckerNextX = new PluckerCoordinate<double>();
    double *a = new double(0); double *alpha = new double(0);
    double *b = new double(0); double *theta = new double(0);;

    std::vector<double> col = {0.0, 0.0, 0.0, 1};
    std::vector<double> colZ = {138.0, 43.0, 226.0, 1.0};
    std::vector<double> colX = {178.0, 34.0, 34.0, 1.0};
    std::vector<double> colE = {255.0, 255.0, 255.0, 1.0};
    std::vector<int> branchesIncludingJoints;
    std::vector<bool> lastJointInBranch; // last joint in branch
    bool isLast;
    for(int c = 0; c < branches.size(); ++c){

      //find the last edge
      std::vector<int>::iterator inx = std::find(branchesEE.begin(), branchesEE.end(), branches[c]);
      if(*inx == branches[c] ){
        isLast = true;
      }else{
        isLast = false;
      }

      // find the last joint in the last edge
      // if branches[c] is 2 then I look at jA[2-1] to get the number of joints in that edge
      for(int z = 0; z < jA[branches[c] - 1]; ++z){
        branchesIncludingJoints.push_back(branches[c]);
        if( (isLast == true) && (z == jA[branches[c] - 1] - 1) ){
          lastJointInBranch.push_back(true);
        }else{
           lastJointInBranch.push_back(false);
        }
      }
    }
    funcs::coutVector("branchesWithJointNumbers: ", branchesIncludingJoints);
    funcs::coutVector("lastJoints:", lastJointInBranch);

    std::string linkP = "edge0_linkP";
    fw << fixedJoint("base_"+linkP, "base", linkP, {0,0,0,0,0,0});
    fw << cylinderLink(linkP, {0,0,0,0,0,0}, {0, 0}, col);
    std::vector<int> childrenIx = findChildrenByParentValue(pA, 0);
    std::string link1Z;
    for(int m = 0; m < childrenIx.size(); ++m){ // go through the children of the current edge
      link1Z = "edge0_link0Z_" + std::to_string(childrenIx[m]+1);
      fw << fixedJoint(linkP+"_"+link1Z, linkP, link1Z, {0,0,0,0,0,0});
    }

    int poseMatrixCounter = 0;
    int axisCounter = 0;
    bool beginingOfEdge, middleOfEdge, endOfEdge, endOfBranch;
    for(int w = 0; w < jointAxesArray.size(); ++w){

      std::vector<double> cl1;
      std::vector<double> cl2;
      *pluckerNextZ = jointAxesArray[w];
      std::vector<double> arP = {0,0,0}; if(w != 0){arP = jointLinkStartPoint[w-1];}
      calculateCommonNormal(*pluckerPreviousZ, *pluckerNextZ, pluckerNextX, a, alpha, cl1, cl2, arP);
      jointLinkStartPoint.push_back(cl2);
      xAxesArray.push_back(*pluckerNextX);

      calculateCommonNormal(*pluckerPreviousX, *pluckerNextX, pluckerTempZ, b, theta, cl1, cl2, arP);
      *pluckerPreviousX = *pluckerNextX;

      // tempZ is pointing from previousZ to nextX while it might not necessarily be in the same direction
      // of the joint axes. compare the tempZ with previousZ, if they are in opposite direction then
      // b and theta should be negative. if they are in the same direction, b and theta should be positive.
      // this is besically the difference between b and a as we define the joint direction in the first place
      // and define the x-Axis direction based on z-axis (joint axis)
      funcs::coutPlucker("************** pluckerPreviousZ ========> ", *pluckerPreviousZ);
      funcs::coutPlucker("************** pluckerPreviousZ ========> ", *pluckerTempZ);
      double same = dotProduct(pluckerPreviousZ->getDirection(), pluckerTempZ->getDirection());
      if(same < 0){
        *b = -*b;
        *theta = -*theta;
      }
      *pluckerPreviousZ = *pluckerNextZ;

      // a joint can be at one of the following:
      // beginingOfEdge
      // middleOfEdge
      // endOfEdge (splitter)
      // endOfEdge (endOfBranch)
      // beginingOfEdge + endOfEdge  (splitter)
      // beginingOfEdge + endOfEdge  (endOfBranch)

      beginingOfEdge = ((branchesIncludingJoints[w] != branchesIncludingJoints[w-1])&& (w>0)) || w == 0;
      middleOfEdge = ((branchesIncludingJoints[w] == branchesIncludingJoints[w-1]) &&
          (branchesIncludingJoints[w] == branchesIncludingJoints[w+1]) && (w>0) && (w<jointAxesArray.size()));
      endOfEdge = ((branchesIncludingJoints[w] != branchesIncludingJoints[w+1]) &&
                   (lastJointInBranch[w] == false) &&(w<jointAxesArray.size()));

      // begining or end of edge can also be end of branch
      endOfBranch = lastJointInBranch[w];

      std::cout << "w => " << w << std::endl;
      std::cout << "begining:" << beginingOfEdge << " middle:" << middleOfEdge << " endOfEdge:" << endOfEdge << " endOfBranch:" << endOfBranch << std::endl;

      // --------------- create link0z and link0x
      std::string link0Z, link0X;
      if(beginingOfEdge ){ // begining or when the root is the parent
        axisCounter = 0;
        int pp = pA[branchesIncludingJoints[w]-1];
        std::string strr ;
        if(pp == 0){ // root is the parent
          strr = std::to_string(0);
        }else{
          strr = std::to_string( jA[pA[branchesIncludingJoints[w]-1]-1]-1  );
        }
        link0Z = "edge" + std::to_string(pA[branchesIncludingJoints[w]-1]) + "_link" + strr+ "Z_" + std::to_string(branchesIncludingJoints[w]);
        link0X = "edge" + std::to_string(pA[branchesIncludingJoints[w]-1]) + "_link" + strr + "X_" + std::to_string(branchesIncludingJoints[w]);
        // jA[pA[branchesIncludingJoints[w]-1]-1]-1 => first and second -1 is becuase index is 1 less than
        // edge number and the last -1 is because I start the joint number from 0: J0, J1 ...
        col[0] = 0;  col[1] =255; col[2] =255; col[3] = 1;
        fw << cylinderLink(link0Z, {0,0,*b/2,0,0,0}, {*b, linkz_thikness}, colZ);
        fw << fixedJoint(link0Z+"_"+link0X, link0Z, link0X, {0,0,*b,0,0,*theta});
        col[0] = 138;  col[1] = 43; col[2] = 226; col[3] = 1;
        fw << cylinderLink(link0X, {*a/2,0,0,0,PI/2,0}, {*a, linkx_thikness}, colZ);
      }else{ // middle or end
        ++axisCounter;
        link0Z = "edge" + std::to_string(branchesIncludingJoints[w]) + "_link" + std::to_string(axisCounter-1) + "Z";
        link0X = "edge" + std::to_string(branchesIncludingJoints[w]) + "_link" + std::to_string(axisCounter-1) + "X";
        fw << cylinderLink(link0Z, {0,0,*b/2,0,0,0}, {*b, linkz_thikness}, colZ);
        fw << fixedJoint(link0Z+"_"+link0X, link0Z, link0X, {0,0,*b,0,0,*theta});
        fw << cylinderLink(link0X, {*a/2,0,0,0,PI/2,0}, {*a, linkx_thikness}, colZ);
      }

      // --------------- create link1z and linkP
      if(endOfEdge){ // (splitter or palm)
        linkP = "edge" + std::to_string(branchesIncludingJoints[w]) + "_linkP";
        fw << revoluteJoint(link0X+"_"+linkP, link0X, linkP, {*a,0,0,*alpha,0,0}, {0,0,1});
        fw << cylinderLink(linkP, {0,0,0,0,0,0}, {0, 0}, col);
        std::vector<int> childsIx = findChildrenByParentValue(pA, branchesIncludingJoints[w]);
        for(int m = 0; m < childsIx.size(); ++m){ // go through the children of the current edge
          link1Z = "edge" + std::to_string(branchesIncludingJoints[w]) + "_link" + std::to_string(axisCounter) + "Z_" + std::to_string(childsIx[m]+1);
          // childsIx[m]+1 => edge = index + 1
          fw << fixedJoint(linkP+"_"+link1Z, linkP, link1Z, {0,0,0,0,0,0});
        }
      }else{ // middle or begining or w = 0
        link1Z = "edge" + std::to_string(branchesIncludingJoints[w]) + "_link" + std::to_string(axisCounter) + "Z";
        fw << revoluteJoint(link0X+"_"+link1Z, link0X, link1Z, {*a,0,0,*alpha,0,0}, {0,0,1});
      }

      // --------- begining or end, the joint can be the end of the branch
      if(lastJointInBranch[w] == true){ // last joint in the branch
/*
        std::vector<double> cOnLine1;
        std::vector<double> cOnLine2;
        // reference configuration, first pose of each branch: eePoseMatrix[each branch][0]
        // eePoseMatrix is already in order, so I just need to increment its index for each lastJoint
        std::vector<double> zeMoment = crossProduct(eePoseMatrix[poseMatrixCounter][0].getPoint(), eePoseMatrix[poseMatrixCounter][0].getZ());
        PluckerCoordinate<double> zePlucker(eePoseMatrix[poseMatrixCounter][0].getZ(), zeMoment);
        *pluckerNextZ = zePlucker;
        std::vector<double> arbitP = {0,0,0}; if(w != 0){arbitP = jointLinkStartPoint[w-1];}
        calculateCommonNormal(*pluckerPreviousZ, *pluckerNextZ, pluckerNextX, a, alpha, cOnLine1, cOnLine2, arbitP);
        calculateCommonNormal(*pluckerPreviousX, *pluckerNextX, pluckerTempZ, b, theta, cOnLine1, cOnLine2, arbitP);
        *pluckerPreviousX = *pluckerNextX;

        // we should check if be and theta should be negative or not
        double same = dotProduct(pluckerPreviousZ->getDirection(), pluckerTempZ->getDirection());
        if(same < 0){
          *b = -*b;
          *theta = -*theta;
        }
        *pluckerPreviousZ = *pluckerNextZ;

        std::string lk0Z = "edge" + std::to_string(branchesIncludingJoints[w]) + "_link" + std::to_string(axisCounter) + "Z";
        std::string lk0X = "edge" + std::to_string(branchesIncludingJoints[w]) + "_link" + std::to_string(axisCounter) + "X";
        std::string lkZe = "edge" + std::to_string(branchesIncludingJoints[w]) + "_link_Ze";

        fw << cylinderLink(link1Z, {0,0,*b/2,0,0,0}, {*b, linkz_thikness}, colZ);
        fw << fixedJoint(lk0Z+"_"+lk0X, lk0Z, lk0X, {0,0,*b,0,0,*theta});
        fw << cylinderLink(lk0X, {*a/2,0,0,0,PI/2,0}, {*a, linkx_thikness}, colX);
        fw << fixedJoint(lk0X+"_"+lkZe, lk0X, lkZe, {*a,0,0,*alpha,0,0});

        // last screw motion on Z axis towards the origin of the desired pose
        std::vector<double> xeMoment = crossProduct(eePoseMatrix[poseMatrixCounter][0].getPoint(), eePoseMatrix[poseMatrixCounter][0].getX());
        *pluckerNextX = PluckerCoordinate<double>(eePoseMatrix[poseMatrixCounter][0].getX(), xeMoment);
        std::vector<double> arbitraryP = {0,0,0}; if(w != 0){arbitraryP = jointLinkStartPoint[w];}
        calculateCommonNormal(*pluckerPreviousX, *pluckerNextX, pluckerTempZ, b, theta, cOnLine1, cOnLine2, arbitraryP);

        // we should check if be and theta should be negative or not
        same = dotProduct(pluckerPreviousZ->getDirection(), pluckerTempZ->getDirection());
        if(same < 0){
          *b = -*b;
          *theta = -*theta;
        }
        fw << cylinderLink(lkZe, {0,0,*b/2,0,0,0}, {*b, linkz_thikness}, colZ);
*/

        // straignt link from the frame of the last joint to ee
        tf2::Vector3 jointX(pluckerNextX->getDirection()[0], pluckerNextX->getDirection()[1], pluckerNextX->getDirection()[2]);
        tf2::Vector3 jointZ(pluckerNextZ->getDirection()[0], pluckerNextZ->getDirection()[1], pluckerNextZ->getDirection()[2]);
        tf2::Vector3 jointY = tf2::tf2Cross(jointZ, jointX);
        tf2::Matrix3x3 jointMat(jointX[0], jointY[0], jointZ[0],
                                jointX[1], jointY[1], jointZ[1],
                                jointX[2], jointY[2], jointZ[2]);
        tf2::Matrix3x3 eeMat(eePoseMatrix[poseMatrixCounter][0].getX()[0], eePoseMatrix[poseMatrixCounter][0].getY()[0], eePoseMatrix[poseMatrixCounter][0].getZ()[0],
                             eePoseMatrix[poseMatrixCounter][0].getX()[1], eePoseMatrix[poseMatrixCounter][0].getY()[1], eePoseMatrix[poseMatrixCounter][0].getZ()[1],
                             eePoseMatrix[poseMatrixCounter][0].getX()[2], eePoseMatrix[poseMatrixCounter][0].getY()[2], eePoseMatrix[poseMatrixCounter][0].getZ()[2]);
        tf2::Matrix3x3 eeMat_inJointFrame = jointMat.transpose() * eeMat;
        std::vector<double> o2ee = eePoseMatrix[poseMatrixCounter][0].getPoint();
        tf2::Vector3 j2ee(o2ee[0] - jointLinkStartPoint[w][0], o2ee[1] - jointLinkStartPoint[w][1], o2ee[2] - jointLinkStartPoint[w][2]); // ee origin with respect to joint frame
        tf2::Vector3 lastLink_Z = jointMat.transpose() * j2ee; // ee origin with respect to joint frame and expressed in it
        lastLink_Z.normalize();
        tf2::Vector3 lastLink_X(-lastLink_Z[1]/lastLink_Z[0], 1, 0); // dot product of any point on the plane and the z direction (plane normal) is zero
        lastLink_X.normalize();
        tf2::Vector3 lastLink_Y = tf2::tf2Cross(lastLink_Z, lastLink_X);
        lastLink_Y.normalize();
     /*   std::cout << tf2::tf2Dot(lastLink_Y, lastLink_Z )<< std::endl;
        std::cout << lastLink_X.length() << std::endl;
        std::cout << lastLink_Y.length() << std::endl;
        std::cout << lastLink_Z.length() << std::endl;*/
        tf2::Matrix3x3 lastLink_Orientation(lastLink_X[0], lastLink_Y[0], lastLink_Z[0],
                                            lastLink_X[1], lastLink_Y[1], lastLink_Z[1],
                                            lastLink_X[2], lastLink_Y[2], lastLink_Z[2]);
        std::cout << "determinant " << lastLink_Orientation.determinant() << std::endl;
        double roll, pitch, yaw;
        lastLink_Orientation.getRPY(roll, pitch, yaw);
      // std::cout << "roll " << roll << " pitch " << pitch << " yaw " << yaw << std::endl;
        tf2::Vector3 lastLink_Position = (j2ee.length()/2) * lastLink_Z;
        fw << cylinderLink(link1Z, {lastLink_Position[0], lastLink_Position[1], lastLink_Position[2],
                                    roll, pitch, yaw}, {j2ee.length(), linkz_thikness}, colE);

        //---------------------------------------------------------------------
        // we need to find the split where we have branches having same parent
        if(w < jointAxesArray.size() - 1){ // if the current joint is not the last one in the
          // jointAxesArray list, find the edge number of the parent of the next joint in parentArray
          int pE = pA[branchesIncludingJoints[w+1] - 1];
          // find the index of the last joint of this edge (this edge = is the parent) in jointAxesArray
          int lJ = 0;
          for(int x = 0; x < branches.size(); ++x){
            lJ = jA[branches[x]-1] + lJ;
            if(branches[x] == pE){
              break;
            }
          }
          // jointAxesArray start with index 0 and lJ is the number of joints
          if(lJ == 0){
           *pluckerPreviousZ = refZ;
           // on the same index
           *pluckerPreviousX = refX;
          }else {
            lJ = lJ - 1;
            // jointAxesArray start with index 0 and lJ is the number of joints
           *pluckerPreviousZ = jointAxesArray[lJ];
           // on the same index
           *pluckerPreviousX = xAxesArray[lJ];
          }
        }
        //-----------------------------------------------------------------------

        ++poseMatrixCounter;
      }
    }

    fw << "</robot>";
  }
  fw.close();
}


