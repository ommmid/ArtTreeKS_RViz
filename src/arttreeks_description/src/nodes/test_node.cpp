#include <ros/ros.h>
#include "arttreeks_description/node.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

//  std::vector<int> parentArray = {0,0,1,1,2,2,6,6,6,8,8};
 // std::vector<int> jointArray = {2,4,4,4};

  std::vector<int> parentArray = {0 , 1, 1, 2, 2 ,3, 3  } ;
  std::vector<int> jointArray = {1,  1, 1, 1, 1, 1, 2  };

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

  //------------------------ recursive ---------------------------------------
  std::vector<int> edgeOrder = {0};
  Node robotTree(0, parentArray, edgeOrder);
  std::cout << "--------- node information ----------" << std::endl;
  std::cout << "edge number: " << robotTree.data << std::endl;
  std::cout << "the size of children: " << robotTree.children.size() << std::endl;
  std::cout << "is it an edge-node: " << std::to_string( robotTree.isTCP )<< std::endl;

  Node node = robotTree.children[0].children[0];
  std::cout << "--------- node information ----------" << std::endl;
  std::cout << "edge number: " << node.data << std::endl;
  std::cout << "the size of children: " << node.children.size() << std::endl;
  std::cout << "is it an edge-node: " << std::to_string( node.isTCP )<< std::endl;

  for(int i = 0; i < edgeOrder.size(); ++i){
    std::cout << edgeOrder[i] << std::endl;
  }

}
