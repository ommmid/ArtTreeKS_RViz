#include <ros/ros.h>
#include "arttreeks_description/node.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

  std::vector<int> parentArray = {0,1,1,1};
  std::vector<int> jointArray = {2,4,4,4};

  Node tree(0);

  Node *tempNode = new Node();

  tempNode->data = 2;
  tree.children.push_back(*tempNode);

  tempNode->data = 4;
  tree.children.push_back(*tempNode);

  tempNode->data = 7;
  tree.children[0].children.push_back(*tempNode);

  std::cout << tree.children[1].data << std::endl;
  std::cout << tree.children[0].children[0].data << std::endl;
}
