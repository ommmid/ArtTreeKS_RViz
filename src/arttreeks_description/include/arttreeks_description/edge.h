#ifndef EDGE_H
#define EDGE_H

#include <arttreeks_description/pluckercoordinate.h>
#include <vector>

struct edge
{
public:
  edge();
  edge(const int& numJ, std::vector<PluckerCoordinate<double>> jAIEdge, std::vector<std::vector<double>> jV);
  int getNumberOFJoints();
  std::vector<PluckerCoordinate<double>> getJointAxesInEdge();
  std::vector<std::vector<double>> getJointValues();

private:
  int numberOfJoints;
  std::vector<std::vector<double>> jointValues; // each edge has some joint axes and each joint has some values
  std::vector<PluckerCoordinate<double>> jointAxesInEdge;
};

#endif // EDGE_H
