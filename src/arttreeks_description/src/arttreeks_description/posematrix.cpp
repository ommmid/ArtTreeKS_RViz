#include "arttreeks_description/posematrix.h"

PoseMatrix::PoseMatrix(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z)
{
  mx.push_back(x); mx.push_back(y); mx.push_back(z);
}


std::vector<double> PoseMatrix::getX(){
  return std::vector<double>{mx[0][0], mx[1][0], mx[2][0]};
}

std::vector<double> PoseMatrix::getY(){
  return std::vector<double>{mx[0][1], mx[1][1], mx[2][1]};
}

std::vector<double> PoseMatrix::getZ(){
  return std::vector<double>{mx[0][2], mx[1][2], mx[2][2]};
}

std::vector<double> PoseMatrix::getPoint(){
  return std::vector<double>{mx[0][3], mx[1][3], mx[2][3]};
}
