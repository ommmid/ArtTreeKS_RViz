#include <ros/ros.h>
#include <arttreeks_description/gnuplot.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>

int xCounter;
int tcpLevel;
bool isTopicON = false;

void chatterCallback(const std_msgs::Int8MultiArrayConstPtr &msg)
{
  isTopicON = true;
  xCounter = msg.get()->data[0] ;
  tcpLevel = msg.get()->data[1] ;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "plot_tree");
  ros::NodeHandle nh;

 ros::Subscriber xCounter_sub = nh.subscribe("robot_info", 1000, chatterCallback);

 ros::Rate r(30);

 for(int i = 0; i < 50; ++i){
   ros::spinOnce();
   std::cout << "collecting data, hold on!" << std::endl;
   r.sleep();
 }

 if(isTopicON){
   gnuplot gplot;
   std::string sstr = "set xrange [-1:" + std::to_string(xCounter) + "] ; set yrange [-2:" + std::to_string(tcpLevel+1) + "];"
       "plot 'points.dat' using 1:2 with lines lc rgb \"black\" lw 2 notitle,"
   " 'points.dat' using 4:5 with circles linecolor rgb \"white\" lw 2 fill solid border lc lt 0 notitle,"
   " 'points.dat' using 1:2:3 with labels offset (0,0) font 'Arial Bold' notitle";

   gplot(sstr);


   std::cin.get();
 }else{
   std::cout << "!!! run create_urdf node first !!!" <<  std::endl;
 }

}
