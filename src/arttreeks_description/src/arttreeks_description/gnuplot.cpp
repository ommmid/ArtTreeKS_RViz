#include "arttreeks_description/gnuplot.h"
#include <string>
#include <vector>
#include <string>
#include <stdio.h>
#include <iostream>

using namespace std;


gnuplot::gnuplot() {
// with -persist option you will see the windows as your program ends
//gnuplotpipe=_popen("gnuplot -persist","w");
//without that option you will not see the window

 // because I choose the terminal to output files so I don't want to see the window

 gnuplotpipe = popen("gnuplot","w");

 if (!gnuplotpipe) {
   std::cerr<< ("Gnuplot not found !");
 }
 else {
   std::cout << "gunplot runs successfully" << std::endl;
 }
}
gnuplot::~gnuplot() {
  fprintf(gnuplotpipe,"exit\n");
  pclose(gnuplotpipe);
}

void gnuplot::operator()(const string & command) {
  fprintf(gnuplotpipe,"%s\n",command.c_str());
  fflush(gnuplotpipe);
// flush is necessary, nothing gets plotted else
}
