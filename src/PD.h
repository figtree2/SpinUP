#ifndef PD_H
#define PD_H

#include "main.h"


class PD {
public:
  double kp;
  double ki;
  double kd;
  double minspeed;
  int counter;

  double prev_error;
  int prev_time;
  double D;
  int counter_reset;

  PD(double p, double d, double min=0, int counter=100);

  double get_value(double error);
  void reset();
};

#endif
