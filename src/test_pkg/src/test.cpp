#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <string>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;


  std::vector<int> vec = {1,2,3,4,5,6};
  vec.insert(vec.begin()+2, 10);
  for(int i = 0; i < vec.size(); ++i){
    std::cout << vec[i] << std::endl;
  }

  std::cout << "-------------------" << std::endl;


  std::vector<int>::iterator it;
  std::vector<int>::iterator index;
  for(it = vec.begin(); it != vec.end(); ++it){
    //std::cout << it << std::endl;
    if(*it == 10){
      index = it;
    }
  }
  std::cout << *index << std::endl;

  std::cout << vec[2] << std::endl;
}
