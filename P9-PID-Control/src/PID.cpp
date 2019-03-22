#include "PID.h"


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  sum_cte = 0;
  previous_cte = 0;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  i_error = cte + sum_cte;
  d_error = cte - previous_cte;
  previous_cte = cte;
}

double PID::TotalError() {
  // P, D, I
  double total_error = - Kp*p_error - Kd*d_error - Ki*i_error;
  return total_error;
}