#ifndef NODE_H
#define NODE_H

#include <vector>

class Node
{
public:
  std::vector<Node> children;
  int data; // number of joints in the previous edge

  Node();
  Node(int data);
  ~Node();

private:

};



#endif // NODE_H
