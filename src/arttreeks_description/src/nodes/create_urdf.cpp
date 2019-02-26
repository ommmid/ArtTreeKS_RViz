#include <ros/ros.h>
#include <fstream>
#include <arttreeks_description/robot.h>
#include <vector>
#include <stdio.h>
#include "arttreeks_description/gnuplot.h"
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <arttreeks_description/funcs.h>

// the input of this program is the output of the ArtTreeKS and the output is an URDF

int main(int argc, char **argv)
{
  ros::init(argc, argv, "create_urdf");
  ros::NodeHandle n;

  // ------------------------------------------------
  // read the input/output of ArtTreeKS
  std::string inputFileName = "/home/omid/ArtTreeKS/New/ArtTreeKS-master/examples/TestElbow.lua"; // including address
  std::string outputFileName = "/home/omid/ArtTreeKS/New/ArtTreeKS-master/examples/InputOutput/TestElbow_Out.lua"; // including address

  Robot rob(inputFileName, outputFileName);

// rob.plotTree();

  // write to xacro file
  std::string xacroFile = "/home/omid/ros_ws/arttreeks_rviz/src/arttreeks_description/urdf/creator_output.xacro";
  rob.createURDF(xacroFile, 1.5, 1);

  // ------------------------------------------------

  // ?? publish joint axes and joint angles from ArtTreeKS_OUTput

  ros::Publisher j_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
  ros::Rate r(30);

  while(ros::ok())
  {
    visualization_msgs::MarkerArray jointsArray;
    for(int i = 0; i < rob.getJointAxesArray().size(); ++i){
      PluckerCoordinate<double> axis = rob.getJointAxesArray()[i]; //funcs::coutPlucker("axissssss: ",axis);
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
      geometry_msgs::Point pl_point;// = funcs::crossProductPoints(pl_direction, pl_moment);
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

    r.sleep();
  }

 // std::cin.get();
}



