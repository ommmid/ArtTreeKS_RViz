#include "arttreeks_description/node.h"
#include <vector>
#include <iostream>


Node::Node(){

}

Node::Node(int edgeNumber)
{
  this->data = edgeNumber;
}

// parentArray = {0,1,1,1}, Index = {0,1,2,3}, edgeNumber = {1,2,3,4}
// pA = {0 , 1, 1, 2, 2 ,3, 3  }
Node::Node(const int &edgeNumber, const std::vector<int> &parentArray, std::vector<int> &edgeOrder)
{
  std::cout << "---------------------------------------------------" << std::endl;
  std::cout << "--------------------------------- -----------------" << std::endl;
  isTCP = false;
  this->data = edgeNumber;
  std::cout << "edge number: " << edgeNumber << std::endl;

  // ------------- find children indexes in parentArray
  std::vector<int> childrenIndex;
  // find children by the parent index in parentArray
  for(int i = 0; i < parentArray.size(); ++i){
   // std::cout << "i ==> " << i << std::endl;
    if( edgeNumber == parentArray[i] ){ // edge including edge number 0
      childrenIndex.push_back(i);
      std::cout << "childrenIndex: " << i << std::endl;
    }
  }
  //std::cout << "childrenIndex.size() " << childrenIndex.size() << std::endl;

  if(childrenIndex.size() == 0){
    isTCP = true;
  }

 // std::cout << "-------------------------------------------------------" << std::endl;
 // std::cout << "---------------- update edgeOrder ---------------------" << std::endl;

  // find the parent in edgeOrder array and insert all the children after that
 /* std::vector<int>::iterator parentIndex;
  for(std::vector<int>::iterator it = edgeOrder.begin(); it != edgeOrder.end(); ++it){
    if(data == *it){
      parentIndex = it;
    }
  }parentIndex as an iterator creates problem that I could not figure out
  I use int instead */
  int parentIndex;
  for(int q = 0; q < edgeOrder.size(); ++q){
    if(data == edgeOrder[q]){
      parentIndex = q;
    }
  }

  // if parentIndex is pointing to the last element of edgeOrder vector, then we should
  // push_back the children
  if(parentIndex == edgeOrder.size()-1 ){
    for(int t = 0; t < childrenIndex.size(); ++t){
      edgeOrder.push_back(childrenIndex[t]+1);
    }
  }else{ // otherwise the children should be inserted
    for(int t = 0; t < childrenIndex.size(); ++t){
      edgeOrder.insert(edgeOrder.begin() + parentIndex+t+1, childrenIndex[t]+1);
    }
  }
 // The vector's storage got reallocated (to accommodate more elements in one contiguous block),
  // invalidating all the pointers

 // std::cout << "---------------------------------------------------" << std::endl;
 // std::cout << "---------------------- edge order -----------------" << std::endl;
  std::cout << "the size of edgeOrder=> " << edgeOrder.size() << " :" << std::endl;
  for(int i = 0; i < edgeOrder.size(); ++i){
    std::cout << edgeOrder[i] << std::endl;
  }


  //-------------- make nodes out of children and out them in the children vec
//  std::cout << "------------------------------------------------------------" << std::endl;
//  std::cout << "---------------------- make children nodes -----------------" << std::endl;
  for(int j = 0; j < childrenIndex.size(); ++j){
    Node node(childrenIndex[j]+1, parentArray, edgeOrder);
    children.push_back(node);
  }

}


Node::~Node(){

}
