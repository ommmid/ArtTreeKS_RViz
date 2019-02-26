#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <math.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <arttreeks_description/funcs.h>

const float PI = 3.14159265359;


int main(int argc, char **argv)
{
  // ------------------------- make dual quaternion by q and v
  tf2::Quaternion orientation; orientation.setRPY(0,0,0.9);
  tf2::Vector3 position(2,3,4);
  std::vector<double> dquat = funcs::makeDQ_qv(orientation, position);
  funcs::coutVector("dquat ======> ", dquat);

  // ------------------------- exctract q and v from a qual quaternion
  tf2::Quaternion *q = new tf2::Quaternion();
  tf2::Vector3 *v = new tf2::Vector3;
  funcs::extract_qv(dquat, q, v);
  ROS_INFO_STREAM("angle: " << q->getAngle() << "    axis: "
                  << q->getAxis().x() << " " << q->getAxis().y() << " " << q->getAxis().z() );
  ROS_INFO_STREAM( "position: " <<  v->x() << " " << v->y() << " " << v->z()  );

  // -------------------------make 5 dual quaternions for an RRR synthesis test
  std::vector<std::vector<double>> dqs;
  std::vector<tf2::Quaternion> qs;
  std::vector<tf2::Vector3> vs;

  double angle1 = 63*PI/180; tf2::Vector3 axis1 = {1,5,1};
  tf2::Quaternion q1; q1.setRotation(axis1, angle1); qs.push_back(q1);
  tf2::Vector3 v1 = {5,-2,-3}; vs.push_back(v1);
  std::vector<double> dq1 = funcs::makeDQ_qv(q1, v1); dqs.push_back(dq1);

  double angle2 = 105*PI/180; tf2::Vector3 axis2 = {4,2,5};
  tf2::Quaternion q2; q2.setRotation(axis2, angle2); qs.push_back(q2);
  tf2::Vector3 v2 = {2,5,-6}; vs.push_back(v2);
  std::vector<double> dq2 = funcs::makeDQ_qv(q2, v2); dqs.push_back(dq2);

  double angle3 = 85*PI/180; tf2::Vector3 axis3 = {3,2,1};
  tf2::Quaternion q3; q3.setRotation(axis3, angle3); qs.push_back(q3);
  tf2::Vector3 v3 = {2,-4,-2}; vs.push_back(v3);
  std::vector<double> dq3 = funcs::makeDQ_qv(q3, v3); dqs.push_back(dq3);

  double angle4 = 20*PI/180; tf2::Vector3 axis4 = {1,2,3};
  tf2::Quaternion q4; q4.setRotation(axis4, angle4); qs.push_back(q4);
  tf2::Vector3 v4 = {5,-2,3}; vs.push_back(v4);
  std::vector<double> dq4 = funcs::makeDQ_qv(q4, v4); dqs.push_back(dq4);

  double angle5 = 60*PI/180; tf2::Vector3 axis5 = {2,3,7};
  tf2::Quaternion q5; q5.setRotation(axis5, angle5); qs.push_back(q5);
  tf2::Vector3 v5 = {2,-3,4}; vs.push_back(v5);
  std::vector<double> dq5 = funcs::makeDQ_qv(q5, v5); dqs.push_back(dq5);

  std::ofstream fw;
  fw.open("dq.txt");
  if(fw.is_open()){
    for(int j = 0; j < 5; ++j){
      fw << std::to_string(dqs[j][3]) << ", " << std::to_string(dqs[j][0]) << ", " <<
            std::to_string(dqs[j][1]) << ", " << std::to_string(dqs[j][2]) << ", " <<
            std::to_string(dqs[j][4]) << ", " << std::to_string(dqs[j][5]) << ", " <<
            std::to_string(dqs[j][6]) << ", " << std::to_string(dqs[j][7]) << std::endl;
    }
  }
  fw.close();


}

