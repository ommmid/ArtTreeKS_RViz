#include <ros/ros.h>
#include <fstream>
#include <arttreeks_description/robot.h>
#include <vector>
#include <stdio.h>
#include "arttreeks_description/gnuplot.h"
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <arttreeks_description/funcs.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>

// the input of this program is the output of the ArtTreeKS and the output is an URDF

int main(int argc, char **argv)
{
  ros::init(argc, argv, "create_urdf");
  ros::NodeHandle n;

  // --------------------------- read the input/output of ArtTreeKS
  std::string inputFileName = "/home/omid/ArtTreeKS/New/ArtTreeKS-master/examples/TestWrist.lua"; // including address
  std::string outputFileName = "/home/omid/ArtTreeKS/New/ArtTreeKS-master/examples/InputOutput/TestWrist_Out.lua"; // including address

  Robot rob(inputFileName, outputFileName);

  //----------- some input for showing targets in RViz
  double l = 20; // the length of frame axes at target pose
  // the scale of arrows of frame axes at target pose. scale.x is the shaft diameter, and scale.y
  // is the head diameter. If scale.z is not zero, it specifies the head length
  double scaleX = 1; double scaleY = 3; double scaleZ = 5;
  double pointScale = 10; // the scale of the point of frame origin at target pose

  // write to xacro file
  std::string xacroFile = "/home/omid/ros_ws/arttreeks_rviz/src/arttreeks_description/urdf/creator_output.xacro";
  rob.createURDF(xacroFile, 1.5, 1);

  std::vector<std::vector<std::vector<double>>> eeDQ = rob.getEEPoseDualQuaternion();
  std::vector<std::vector<double>> dqs;
  for(int t = 0; t < eeDQ.size(); ++t)  {
    for(int s = 0; s < eeDQ[t].size(); ++s){
      dqs.push_back(eeDQ[t][s]);
    }
  }

  // ------------------------------------------------
  ros::Publisher j_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
  ros::Publisher robot_pub = n.advertise<std_msgs::Int8MultiArray>("robot_info", 10);

  ros::Publisher points_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
  ros::Publisher xAxis_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
  ros::Publisher yAxis_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
  ros::Publisher zAxis_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

  ros::Rate r(30);

  std_msgs::Int8MultiArray robot_msg;
  robot_msg.data.push_back(rob.xLevelCounter);
  robot_msg.data.push_back(rob.tcpLevelCounter);
  robot_msg.data.push_back(rob.numPoses);

  while(ros::ok())
  {
    //------------------ robot info
    robot_pub.publish(robot_msg);

    //------------------- joint axes
    visualization_msgs::MarkerArray jointsArray;
    for(int i = 0; i < rob.getJointAxesArray().size(); ++i){
      PluckerCoordinate<double> axis = rob.getJointAxesArray()[i];
      visualization_msgs::Marker joint;
      joint.header.frame_id = "base";
      joint.header.stamp = ros::Time::now();
      joint.ns = "joints_axes";
      joint.action = visualization_msgs::Marker::ADD;
      joint.type = visualization_msgs::Marker::ARROW;
      geometry_msgs::Point pl_direction;
      pl_direction.x = axis.getDirection()[0];
      pl_direction.y = axis.getDirection()[1];
      pl_direction.z = axis.getDirection()[2];
      geometry_msgs::Point pl_moment;
      pl_moment.x = axis.getMoment()[0];
      pl_moment.y = axis.getMoment()[1];
      pl_moment.z = axis.getMoment()[2];
      geometry_msgs::Point pl_point;
      pl_point.x = rob.jointLinkStartPoint[i][0];
      pl_point.y = rob.jointLinkStartPoint[i][1];
      pl_point.z = rob.jointLinkStartPoint[i][2];
      joint.points.push_back(pl_point);
      geometry_msgs::Point pl_point_end;
      double l = 8;
      pl_point_end.x = pl_point.x + l * pl_direction.x;
      pl_point_end.y = pl_point.y + l * pl_direction.y;
      pl_point_end.z = pl_point.z + l * pl_direction.z;
      joint.points.push_back(pl_point_end);

      joint.id = i;
      //scale.x is the shaft diameter, and scale.y is the head diameter.
      //If scale.z is not zero, it specifies the head length
      joint.scale.x = 5; joint.scale.y = 8; joint.scale.z = 5;
      joint.color.r = 0.0f; joint.color.g = 0.0f; joint.color.b = 255.0f;  joint.color.a = 1.0;

      jointsArray.markers.push_back(joint);
    }
    j_pub.publish(jointsArray);

    //----------------------- targets
    visualization_msgs::MarkerArray pointsArray;
    visualization_msgs::MarkerArray xAxisArray;
    visualization_msgs::MarkerArray yAxisArray;
    visualization_msgs::MarkerArray zAxisArray;

    for(int i = 0; i < dqs.size(); ++i){

      tf2::Quaternion *q = new tf2::Quaternion();
      tf2::Vector3 *v = new tf2::Vector3;
      funcs::extract_qv(dqs[i], q, v);
      tf2::Vector3 xDirection, yDirection, zDirection;
      xDirection = tf2::Matrix3x3(*q).getColumn(0);
      yDirection = tf2::Matrix3x3(*q).getColumn(1);
      zDirection = tf2::Matrix3x3(*q).getColumn(2);
      geometry_msgs::Point pl_point, pl_point_end;

      visualization_msgs::Marker point;
      point.header.frame_id = "base";
      point.header.stamp = ros::Time::now();
      point.ns = "points_ns";
      point.action = visualization_msgs::Marker::ADD;
      point.pose.position.x = v->x(); point.pose.position.y = v->y(); point.pose.position.z = v->z();
      point.pose.orientation.w = q->w();
      point.id = i;
      point.type = visualization_msgs::Marker::SPHERE;
      // POINTS markers use x and y scale for width/height respectively
      point.scale.x = pointScale;
      point.scale.y = pointScale;
      point.scale.z = pointScale;
      // Points are green
      if(i == 0){
        point.color.r = 255.0f; point.color.g = 165.0f; point.color.b = 0.0f;
      }else{
        point.color.r = 0.0f; point.color.g = 1.0f; point.color.b = 0.0f;
      }
      point.color.a = 1.0;
      pointsArray.markers.push_back(point);

      visualization_msgs::Marker xAxis;
      xAxis.header.frame_id = "base";
      xAxis.header.stamp = ros::Time::now();
      xAxis.ns = "xAxis_ns";
      xAxis.action = visualization_msgs::Marker::ADD;
      xAxis.type = visualization_msgs::Marker::ARROW;
      pl_point.x = v->getX(); pl_point.y = v->getY(); pl_point.z = v->getZ();
      xAxis.points.push_back(pl_point);
      pl_point_end.x = pl_point.x + l * xDirection.x();
      pl_point_end.y = pl_point.y + l * xDirection.y();
      pl_point_end.z = pl_point.z + l * xDirection.z();
      xAxis.points.push_back(pl_point_end);
      xAxis.scale.x = scaleX; xAxis.scale.y = scaleY; xAxis.scale.z = scaleZ;
      xAxis.color.r = 1.0f; xAxis.color.g = 0.0f; xAxis.color.b = 0.0f;  xAxis.color.a = 1.0;
      xAxis.id = i;
      xAxisArray.markers.push_back(xAxis);

      visualization_msgs::Marker yAxis;
      yAxis.header.frame_id = "base";
      yAxis.header.stamp = ros::Time::now();
      yAxis.ns = "yAxis_ns";
      yAxis.action = visualization_msgs::Marker::ADD;
      yAxis.type = visualization_msgs::Marker::ARROW;
      pl_point.x = v->getX(); pl_point.y = v->getY(); pl_point.z = v->getZ();
      yAxis.points.push_back(pl_point);
      pl_point_end.x = pl_point.x + l * yDirection.x();
      pl_point_end.y = pl_point.y + l * yDirection.y();
      pl_point_end.z = pl_point.z + l * yDirection.z();
      yAxis.points.push_back(pl_point_end);
      yAxis.scale.x = scaleX; yAxis.scale.y = scaleY; yAxis.scale.z = scaleZ;
      yAxis.color.r = 0.0f; yAxis.color.g = 1.0f; yAxis.color.b = 0.0f;  yAxis.color.a = 1.0;
      yAxis.id = 100 + i;
      yAxisArray.markers.push_back(yAxis);

      visualization_msgs::Marker zAxis;
      zAxis.header.frame_id = "base";
      zAxis.header.stamp = ros::Time::now();
      zAxis.ns = "zAxis_ns";
      zAxis.action = visualization_msgs::Marker::ADD;
      zAxis.type = visualization_msgs::Marker::ARROW;
      pl_point.x = v->getX(); pl_point.y = v->getY(); pl_point.z = v->getZ();
      zAxis.points.push_back(pl_point);
      pl_point_end.x = pl_point.x + l * zDirection.x();
      pl_point_end.y = pl_point.y + l * zDirection.y();
      pl_point_end.z = pl_point.z + l * zDirection.z();
      zAxis.points.push_back(pl_point_end);
      zAxis.scale.x = scaleX; zAxis.scale.y = scaleY; zAxis.scale.z = scaleZ;
      zAxis.color.r = 0.0f; zAxis.color.g = 0.0f; zAxis.color.b = 1.0f;  zAxis.color.a = 1.0;
      zAxis.id = 200 + i;
      zAxisArray.markers.push_back(zAxis);
    }
    points_pub.publish(pointsArray);
    xAxis_pub.publish(xAxisArray);
    yAxis_pub.publish(yAxisArray);
    zAxis_pub.publish(zAxisArray);
    //-----------------------

    r.sleep();
  }

}



