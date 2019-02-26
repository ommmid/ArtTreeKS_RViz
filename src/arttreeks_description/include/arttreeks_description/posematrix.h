#ifndef HOMOGENEOUSMATRIX_H
#define HOMOGENEOUSMATRIX_H

#include <iostream>
#include <vector>

struct PoseMatrix
{
public:
  std::vector<std::vector<double>> mx;
  PoseMatrix(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z); // horizontal vectors in matrix

  std::vector<double> getX();
  std::vector<double> getY();
  std::vector<double> getZ();
  std::vector<double> getPoint();

private:

};

#endif // HOMOGENEOUSMATRIX_H
