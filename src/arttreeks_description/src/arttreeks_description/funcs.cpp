#include "arttreeks_description/funcs.h"
#include <iostream>
#include <vector>
#include <sstream>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

tf2::Vector3 funcs::crossProduct(const tf2::Vector3 &a, const tf2::Vector3 &b){
  tf2::Vector3 p;
  p.setX(a.y() * b.z() - a.z() * b.y() );
  p.setY(a.z() * b.x() - a.x() * b.z() );
  p.setZ(a.x() * b.y() - a.y() * b.x() );

  return p;
}
geometry_msgs::Point funcs::crossProductPoints(const geometry_msgs::Point &a, const geometry_msgs::Point &b){
  geometry_msgs::Point p;
  p.x = (a.y * b.z - a.z * b.y );
  p.y = (a.z * b.x - a.x * b.z );
  p.z = (a.x * b.y - a.y * b.x );

  return p;
}


// q = [qx, qy, qz, w], position v=[x,y,z,0] => dq = q + quat_multiplication(q,v)/2
std::vector<double> funcs::makeDQ_qv(const tf2::Quaternion &q, const tf2::Vector3 &v){
  std::vector<double> dq;
  dq.push_back(q.x()); dq.push_back(q.y()); dq.push_back(q.z()); dq.push_back(q.w());
  tf2::Quaternion v_quaternion(v.x(), v.y(), v.z(), 0);
  tf2::Quaternion qv = v_quaternion * q;

  dq.push_back(qv.x()/2); dq.push_back(qv.y()/2); dq.push_back(qv.z()/2); dq.push_back(qv.w()/2);

 return dq;
}

tf2::Quaternion funcs::quat_conjugate(const tf2::Quaternion &q){
  tf2::Quaternion q_conjugate(-q.x(), -q.y(), -q.z(), q.w());
  return q_conjugate;
}

// extract the quaternion (as the rotation) and the position from a dual quaternion
void funcs::extract_qv(const std::vector<double> &dq, tf2::Quaternion *q, tf2::Vector3 *v){
  std::vector<double> qv(dq.begin(), dq.begin() + 3 + 1); // second argument needs to be one more than the desired index
  q->setX(qv[0]); q->setY(qv[1]); q->setZ(qv[2]); q->setW(qv[3]);

  std::vector<double> d_vector(dq.begin()+4, dq.begin() + 7 + 1);
  tf2::Quaternion d_quaternoin(d_vector[0], d_vector[1], d_vector[2], d_vector[3]);

  tf2::Quaternion v_quaternion = d_quaternoin * quat_conjugate(*q) ;
  v->setX(2*v_quaternion.x());
  v->setY(2*v_quaternion.y());
  v->setZ(2*v_quaternion.z());
}

std::vector<double> funcs::convertString2Vector(const std::string &stringValue){

  std::vector<double> outVector;
  int comma_Index_old = -1;
  int comma_Index_new = stringValue.find(",", comma_Index_old + 1);
  std::string numberString;

  while(comma_Index_new != -1){
    numberString = stringValue.substr(comma_Index_old + 1, comma_Index_new - comma_Index_old);
    std::istringstream os(numberString);
    double num;
    os >> num;
    outVector.push_back(num);
    comma_Index_old = comma_Index_new;
    comma_Index_new = stringValue.find(",", comma_Index_old + 1);
  }
  std::istringstream osEnd(stringValue.substr(comma_Index_old + 1, stringValue.size() - comma_Index_old));
  double numEnd;
  osEnd >> numEnd;
  outVector.push_back(numEnd);

  return outVector;
}


std::vector<double> funcs::vectorMultipliedByDouble(const std::vector<double>& v, const double& d){
  std::vector<double> outV = {v[0]*d, v[1]*d, v[2]*d};
  return outV;
}
std::vector<double> funcs::vectorDividedByDouble(const std::vector<double>& v, const double &d){
  std::vector<double> outV = {v[0]/d, v[1]/d, v[2]/d};
  return outV;
}
double funcs::vectorMagnitude(const std::vector<double> &v){
  return sqrt( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] );
}
double funcs::dotProduct(const std::vector<double> &v1, const std::vector<double> &v2){
  return v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2];
}
std::vector<double> funcs::crossProduct(const std::vector<double> &v1, const std::vector<double> &v2){
  std::vector<double> cross;
  cross.push_back(v1[1]*v2[2]-v1[2]*v2[1]);
  cross.push_back(v1[2]*v2[0]-v1[0]*v2[2]);
  cross.push_back(v1[0]*v2[1]-v1[1]*v2[0]);
  return cross;
}
std::vector<double> funcs::sumVector(const std::vector<double> &v1, const std::vector<double> &v2){
  std::vector<double> outV = {v1[0]+v2[0], v1[1]+v2[1], v1[2]+v2[2]};
  return outV;
}

