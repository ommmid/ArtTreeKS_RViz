#ifndef FUNCS_H
#define FUNCS_H

#include <string>
#include <vector>
#include <iostream>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <arttreeks_description/pluckercoordinate.h>

namespace funcs {

template<typename T>
void coutVector(const std::string &name, const std::vector<T> &vec);

tf2::Vector3 crossProduct(const tf2::Vector3 &a, const tf2::Vector3 &b);
geometry_msgs::Point crossProductPoints(const geometry_msgs::Point &a, const geometry_msgs::Point &b);

std::vector<double> makeDQ_qv(const tf2::Quaternion &q, const tf2::Vector3 &v);

tf2::Quaternion quat_conjugate(const tf2::Quaternion &q);

void extract_qv(const std::vector<double> &dq, tf2::Quaternion *q, tf2::Vector3 *v);

std::vector<double> convertString2Vector(const std::string &stringValue);

std::vector<double> vectorMultipliedByDouble(const std::vector<double>& v, const double& d);
std::vector<double> vectorDividedByDouble(const std::vector<double>& v, const double &d);
double vectorMagnitude(const std::vector<double> &v);
double dotProduct(const std::vector<double> &v1, const std::vector<double> &v2);
std::vector<double> crossProduct(const std::vector<double> &v1, const std::vector<double> &v2);
std::vector<double> sumVector(const std::vector<double> &v1, const std::vector<double> &v2);

template<typename T2>
void coutPlucker(const std::string &name, const PluckerCoordinate<T2> &pl);

}

template<typename T>
void funcs::coutVector(const std::string &name, const std::vector<T> &vec){
  std::string coutString;
  for(int j = 0; j<vec.size(); ++j){
    coutString = coutString + std::to_string(vec[j]) + " " ;
  }
  std::cout << name << " " << coutString << std::endl;
}


template<typename T2>
void funcs::coutPlucker(const std::string &name, const PluckerCoordinate<T2> &pl){
  std::string coutStringDirection;
  for(int j = 0; j<pl.getDirection().size(); ++j){
    coutStringDirection = coutStringDirection + std::to_string(pl.getDirection()[j]) + " " ;
  }

  std::string coutStringMoment;
  for(int j = 0; j<pl.getMoment().size(); ++j){
    coutStringMoment = coutStringMoment + std::to_string(pl.getMoment()[j]) + " " ;
  }
  std::cout << name << " " << "Direction: " << coutStringDirection << " === " << "Moment: " << coutStringMoment << std::endl;
}




#endif // FUNCS_H
