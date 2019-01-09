#include <ros/ros.h>
#include <arttreeks_description/gnuplot.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_gnuplot");
  ros::NodeHandle nh;

  gnuplot plot;
  //plot("plot sin(x)");

  // or reading from a file

  // https://stackoverflow.com/questions/20406346/how-to-plot-tree-graph-web-data-on-gnuplot/20406791

  string str = "plot 'points.dat' using 1:2 with lines lc rgb \"black\" lw 2 notitle,"
  " 'points.dat' using 1:2 with circles linecolor rgb \"white\" lw 2 fill solid border lc lt 0 notitle,"
  " 'points.dat' using 1:2:3 with labels offset (0,0) font 'Arial Bold' notitle";

  plot(str);

std::cin.get();
}
