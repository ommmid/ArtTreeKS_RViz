#ifndef NODE_H
#define NODE_H

#include <vector>

class Node
{
public:
  std::vector<Node> children;
  int data; // number of joints in the previous edge
  bool isTCP;

  //std::vector<int> edgeOrder;

  Node();
  Node(int edgeNumber);
  Node(const int &edgeNumber, const std::vector<int> &parentArray, std::vector<int> &edgeOrder);
  ~Node();

private:
//  void createEdgeOrder(std::vector<int> &v);
};



#endif // NODE_H
