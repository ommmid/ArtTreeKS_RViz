#ifndef GNUPLOT_H
#define GNUPLOT_H

#include <string>
#include <iostream>
using namespace std;

class gnuplot {

public:
  gnuplot();
  ~gnuplot();
  void operator ()(const string & command);
// send any command to gnuplot

protected:
  FILE *gnuplotpipe;
};



#endif // GNUPLOT_H
