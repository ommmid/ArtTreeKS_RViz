#include "arttreeks_description/edge.h"

edge::edge(){

}

edge::edge(const int &numJ, std::vector<PluckerCoordinate<double> > jAIEdge, std::vector<std::vector<double>> jV)
{
  numberOfJoints = numJ;
  jointAxesInEdge = jAIEdge;
  jointValues = jV;
}

int edge::getNumberOFJoints(){
  return numberOfJoints;
}

std::vector<PluckerCoordinate<double>> edge::getJointAxesInEdge(){
  return jointAxesInEdge;
}

std::vector<std::vector<double>> edge::getJointValues(){
  return jointValues;
}
