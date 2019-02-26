#ifndef GNUPLOT_H
#define GNUPLOT_H

#include <iostream>
#include <string>



class gnuplot {

public:
  gnuplot();
  ~gnuplot();
  void operator ()(const std::string& command);
// send any command to gnuplot

protected:
  FILE *gnuplotpipe;
};



#endif // GNUPLOT_H
