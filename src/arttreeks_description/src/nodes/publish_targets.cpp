#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <math.h>
#include <fstream>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <arttreeks_description/funcs.h>
#include <string>

const float PI = 3.14159265359;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "publish_targets");
  ros::NodeHandle n;

  std::string inputFileName = "TestElbow.lua";
  int numberOfPoses = 5;
  double l = 20; // the length of frame axes at target pose
  // the scale of arrows of frame axes at target pose. scale.x is the shaft diameter, and scale.y
  // is the head diameter. If scale.z is not zero, it specifies the head length
  double scaleX = 1; double scaleY = 3; double scaleZ = 5;
  double pointScale = 10; // the scale of the point of frame origin at target pose

  // --------------------------------------- get the poses
  // ---------- read from ArtTreeKS input file
  std::stringstream ss;
  std::ifstream inFile;
  inFile.open("/home/omid/ArtTreeKS/New/ArtTreeKS-master/examples/" + inputFileName);
  if(inFile.is_open()){
    ss << inFile.rdbuf();
    inFile.close();
  }else{
    std::cout << "unable to open the file" << std::endl;
  }
  std::string inString = ss.str();

  std::vector<std::vector<double>> dqs;
  int startingSearchIndex = 0;
  for(int p = 0; p < numberOfPoses; ++p){
    // find the index of the first letter of the first instance of "setPositions( {"
    std::size_t start_index = inString.find("luadq.raw({", startingSearchIndex) + 12; // start index of the search is the
    if(start_index == -1){ std::cout << "check ArtTreeKS output file" << std::endl;  }
    // second argument which is 0 for the first search
    std::size_t end_index = inString.find("}" , start_index);
    std::string subP = inString.substr(start_index, end_index - start_index); // (starting index, size of the substring)
   // std::cout << "sub ===> " << subP << std::endl;

    std::vector<double> dq = funcs::convertString2Vector(subP);
    double qw = dq[0]; double qx = dq[1]; double qy = dq[2]; double qz = dq[3];
    dq[0] = qx; dq[1] = qy; dq[2] = qz; dq[3] = qw;
    //funcs::coutVector("---->", dq);
    dqs.push_back(dq);
    startingSearchIndex = start_index + 10;
  }

  // ---------- read from a file
 /* std::string line;
  std::ifstream inputFile;
  inputFile.open("dq.txt");
  std::vector<std::vector<double>> dqs;
  if(inputFile.is_open()){
    while (std::getline(inputFile, line)) {
      std::vector<double> dq = funcs::convertString2Vector(line);
      dqs.push_back(dq);
    }
    inputFile.close();
  }*/

  // --------------------------------- publish the points as pheres --------------------------
  ros::Publisher points_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
  ros::Publisher xAxis_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
  ros::Publisher yAxis_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
  ros::Publisher zAxis_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
  ros::Rate r(30);

  while(ros::ok())
  //for(int h = 0; h < 1; ++h)
  {
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

    r.sleep();
  }

   // ------------------------- publish pose (arrow or axes) --------------------------

  /*
  ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("pose_topic", 10);
  ros::Rate r(30);

  while(ros::ok())
  {
   // geometry_msgs::PoseArray posesArray;

    for(int i = 0; i < qs.size(); ++i){
    //int i =1;
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "base";
      //pose.pose = geometry_msgs::PoseStamped::; pose.col
      pose.header.stamp = ros::Time::now();
      pose.pose.position.x = vs[i][0]; pose.pose.position.y = vs[i][1]; pose.pose.position.z = vs[i][2];
      pose.pose.orientation.x = qs[i][0];
      pose.pose.orientation.y = qs[i][1];
      pose.pose.orientation.z = qs[i][2];
      pose.pose.orientation.w = qs[i][3];

      pub.publish(pose);
    }

    r.sleep();

  }

  */
  // ------------------------- publish marker arrow as axes --------------------------
 /*
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    ros::Rate r(30);

    while(ros::ok())
    {
      visualization_msgs::MarkerArray xAxisArray;
      visualization_msgs::MarkerArray yAxisArray;
      visualization_msgs::MarkerArray zAxisArray;

      for(int i = 0; i < qs.size(); ++i){
        visualization_msgs::Marker xAxis;
        xAxis.type = visualization_msgs::Marker::ARROW;
        xAxis.action = visualization_msgs::Marker::ADD;
        xAxis.header.frame_id = "base";
        xAxis.header.stamp = ros::Time::now();
        xAxis.ns = "X_Axes";
        xAxis.pose.position.x = vs[i][0]; xAxis.pose.position.y = vs[i][1]; xAxis.pose.position.z = vs[i][2];
        xAxis.pose.orientation.x = qs[i][0];
        xAxis.pose.orientation.y = qs[i][1];
        xAxis.pose.orientation.z = qs[i][2];
        xAxis.pose.orientation.w = qs[i][3];
        xAxis.id = i;
        xAxis.scale.x = 1; xAxis.scale.y = 0.1; xAxis.scale.z = 0.1;
        xAxis.color.a = 1.0;  xAxis.color.r = 1.0f; xAxis.color.g = 0.0f;  xAxis.color.b = 0.0f;
        xAxisArray.markers.push_back(xAxis);

        visualization_msgs::Marker yAxis;
        yAxis.type = visualization_msgs::Marker::ARROW;
        yAxis.action = visualization_msgs::Marker::ADD;
        yAxis.header.frame_id = "base";
        yAxis.header.stamp = ros::Time::now();
        yAxis.ns = "Y_Axes";

        float arrowLength = 1.0f;
        geometry_msgs::Point pS; pS.x = vs[i][0]; pS.y = vs[i][1]; pS.z = vs[i][2];
        geometry_msgs::Point pE;
        pE.x = vs[i][0] + arrowLength*; pE.y = vs[i][1]; pE.z = vs[i][2];

        yAxis.pose.position.x = vs[i][0]; yAxis.pose.position.y = vs[i][1]; yAxis.pose.position.z = vs[i][2];
        yAxis.pose.orientation.x = qs[i][0];
        yAxis.pose.orientation.y = qs[i][1];
        yAxis.pose.orientation.z = qs[i][2];
        yAxis.pose.orientation.w = qs[i][3];
        yAxis.id = i;
        yAxis.scale.x = 1; yAxis.scale.y = 0.1; yAxis.scale.z = 0.1;
        yAxis.color.a = 1.0;  yAxis.color.r = 0.0f; yAxis.color.g = 1.0f;  yAxis.color.b = 0.0f;
        yAxisArray.markers.push_back(yAxis);

        visualization_msgs::Marker zAxis;
        zAxis.type = visualization_msgs::Marker::ARROW;
        zAxis.action = visualization_msgs::Marker::ADD;
        zAxis.header.frame_id = "base";
        zAxis.header.stamp = ros::Time::now();
        zAxis.ns = "Z_Axes";
        zAxis.pose.position.x = vs[i][0]; zAxis.pose.position.y = vs[i][1]; zAxis.pose.position.z = vs[i][2];
        zAxis.pose.orientation.x = qs[i][0];
        zAxis.pose.orientation.y = qs[i][1];
        zAxis.pose.orientation.z = qs[i][2];
        zAxis.pose.orientation.w = qs[i][3];
        zAxis.id = i;
        zAxis.scale.x = 1; zAxis.scale.y = 0.1; zAxis.scale.z = 0.1;
        zAxis.color.a = 1.0;  zAxis.color.r = 0.0f; zAxis.color.g = 0.0f;  zAxis.color.b = 1.0f;
        zAxisArray.markers.push_back(zAxis);
      }

     // marker_pub.publish(xAxisArray);
      marker_pub.publish(yAxisArray);
      marker_pub.publish(zAxisArray);

      r.sleep();

    }
*/

}

